/*
 * Old-map-first scan-matching diagnostic for slam_toolbox.
 *
 * This mapper keeps upstream Karto loop closure intact, but changes the
 * sequential/local scan-matching reference grid when explicitly enabled.
 * Earlier scans contribute more strongly than later scans, while the first
 * pose remains hard-fixed by the existing Ceres solver behavior.
 *
 * The enabled path mirrors Karto's normal AddEdges topology, with one intended
 * difference: local near-chain matching uses the same temporal weighted
 * correlation rule as the initial sequential match. Loop-closure matching is
 * deliberately left upstream/unweighted.
 */

#ifndef SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_
#define SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_

#include <algorithm>
#include <cmath>
#include <iterator>
#include <list>
#include <set>
#include <vector>

#include "karto_sdk/Mapper.h"

namespace mapper_utils
{

class OldMapMapper : public karto::Mapper
{
public:
  OldMapMapper()
  : karto::Mapper(),
    oldmap_scan_weighting_enabled_(false),
    oldmap_scan_min_confidence_(0.25),
    oldmap_scan_decay_nodes_(70.0),
    oldmap_keep_first_scan_(true),
    oldmap_keyframe_distance_(0.5),
    oldmap_history_search_radius_(3.0),
    oldmap_history_max_keyframes_(40)
  {
  }

  void setOldMapScanWeightingEnabled(bool enabled)
  {
    oldmap_scan_weighting_enabled_ = enabled;
  }

  void setOldMapScanMinConfidence(double value)
  {
    oldmap_scan_min_confidence_ = std::max(0.01, std::min(1.0, value));
  }

  void setOldMapScanDecayNodes(double value)
  {
    oldmap_scan_decay_nodes_ = std::max(1.0, value);
  }

  void setOldMapKeepFirstScan(bool enabled)
  {
    oldmap_keep_first_scan_ = enabled;
  }

  void setOldMapKeyframeDistance(double value)
  {
    oldmap_keyframe_distance_ = std::max(0.0, value);
  }

  void setOldMapHistorySearchRadius(double value)
  {
    oldmap_history_search_radius_ = std::max(0.1, value);
  }

  void setOldMapHistoryMaxKeyframes(int value)
  {
    oldmap_history_max_keyframes_ = std::max(1, value);
  }

  kt_bool Process(karto::LocalizedRangeScan * pScan, karto::Matrix3 * covariance = nullptr) override
  {
    if (!oldmap_scan_weighting_enabled_) {
      return karto::Mapper::Process(pScan, covariance);
    }

    if (pScan == nullptr) {
      return false;
    }

    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();
    if (pLaserRangeFinder == nullptr || !pLaserRangeFinder->Validate(pScan)) {
      return false;
    }

    if (!m_Initialized) {
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    karto::LocalizedRangeScan * pLastScan =
      m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // Preserve upstream odometry-to-corrected-pose propagation before matching.
    if (pLastScan != nullptr) {
      karto::Transform lastTransform(
        pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    if (!HasMovedEnough(pScan, pLastScan)) {
      return false;
    }

    karto::Matrix3 cov;
    cov.SetToIdentity();

    // Initial sequential/local correction uses recent scans plus trusted
    // spatially relevant historical keyframes, with old > new confidence.
    if (m_pUseScanMatching->GetValue() && pLastScan != nullptr) {
      karto::Pose2 bestPose;
      karto::LocalizedRangeScanVector references;
      std::vector<kt_double> weights;
      BuildReferenceSet(pScan, references, weights);

      if (!references.empty()) {
        WeightedMatchScan(pScan, references, weights, bestPose, cov, true);
      } else {
        // Defensive fallback: preserve upstream behavior if no weighted
        // reference can be constructed for an otherwise valid scan.
        m_pSequentialScanMatcher->MatchScan(
          pScan,
          m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
          bestPose,
          cov);
      }

      pScan->SetSensorPose(bestPose);
      if (covariance != nullptr) {
        *covariance = cov;
      }
    }

    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      m_pGraph->AddVertex(pScan);

      // Mirror Karto AddEdges, but route LinkNearChains through the weighted
      // old-map-first matcher so every local matching stage follows the same
      // temporal hierarchy. Loop closure below remains untouched.
      AddEdgesOldMap(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<karto::Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
        for (const karto::Name & sensorName : deviceNames) {
          m_pGraph->TryCloseLoop(pScan, sensorName);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);
    return true;
  }

private:
  kt_double RawConfidence(const karto::LocalizedRangeScan * scan) const
  {
    const int age = std::max(0, static_cast<int>(scan->GetStateId()));
    const kt_double alpha = std::exp(
      -static_cast<kt_double>(age) / oldmap_scan_decay_nodes_);
    return oldmap_scan_min_confidence_ +
           (1.0 - oldmap_scan_min_confidence_) * alpha;
  }

  void NormalizeRawWeights(
    const std::vector<kt_double> & raw_weights,
    std::vector<kt_double> & normalized_weights) const
  {
    normalized_weights.clear();
    if (raw_weights.empty()) {
      return;
    }

    // Normalize by the strongest active reference. The oldest active scan gets
    // weight 1.0, while later scans keep the strict temporal ordering below it.
    // This avoids globally shrinking match responses when only late scans are
    // available in a newly explored region.
    const kt_double max_raw = *std::max_element(raw_weights.begin(), raw_weights.end());
    const kt_double denominator = std::max<kt_double>(max_raw, 1e-6);
    normalized_weights.reserve(raw_weights.size());
    for (kt_double raw : raw_weights) {
      normalized_weights.push_back(
        std::max<kt_double>(0.01, std::min<kt_double>(1.0, raw / denominator)));
    }
  }

  void BuildWeightsForReferences(
    const karto::LocalizedRangeScanVector & references,
    std::vector<kt_double> & normalized_weights) const
  {
    std::vector<kt_double> raw_weights;
    raw_weights.reserve(references.size());
    for (karto::LocalizedRangeScan * scan : references) {
      raw_weights.push_back(scan != nullptr ? RawConfidence(scan) : 0.01);
    }
    NormalizeRawWeights(raw_weights, normalized_weights);
  }

  void AddReference(
    karto::LocalizedRangeScan * scan,
    std::set<karto::LocalizedRangeScan *> & seen,
    karto::LocalizedRangeScanVector & references,
    std::vector<kt_double> & raw_weights) const
  {
    if (scan == nullptr || seen.find(scan) != seen.end()) {
      return;
    }
    seen.insert(scan);
    references.push_back(scan);
    raw_weights.push_back(RawConfidence(scan));
  }

  void BuildReferenceSet(
    karto::LocalizedRangeScan * pScan,
    karto::LocalizedRangeScanVector & references,
    std::vector<kt_double> & normalized_weights) const
  {
    references.clear();
    normalized_weights.clear();

    std::vector<kt_double> raw_weights;
    std::set<karto::LocalizedRangeScan *> seen;

    // Always retain the normal recent running buffer. This preserves local
    // continuity even when the robot is far from all historical keyframes.
    const karto::LocalizedRangeScanVector & running =
      m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName());
    for (karto::LocalizedRangeScan * scan : running) {
      AddReference(scan, seen, references, raw_weights);
    }

    // Add sparse trusted historical keyframes near the predicted current pose.
    // Keyframes are chosen in chronological order, so if many old passes are
    // nearby the earliest observations are retained first.
    const karto::Pose2 predicted_pose = pScan->GetSensorPose();
    karto::LocalizedRangeScanMap & all_scans =
      m_pMapperSensorManager->GetScans(pScan->GetSensorName());

    karto::LocalizedRangeScan * last_keyframe = nullptr;
    int historical_added = 0;
    const kt_double radius_sq =
      oldmap_history_search_radius_ * oldmap_history_search_radius_;

    for (auto & item : all_scans) {
      karto::LocalizedRangeScan * scan = item.second;
      if (scan == nullptr) {
        continue;
      }

      bool is_keyframe = false;
      if (scan->GetStateId() == 0) {
        // State 0 defines the keyframe-spacing origin even when the direct
        // first-scan reference is disabled.
        is_keyframe = oldmap_keep_first_scan_;
        last_keyframe = scan;
      } else if (last_keyframe == nullptr) {
        is_keyframe = true;
        last_keyframe = scan;
      } else {
        const kt_double spacing_sq = scan->GetSensorPose().GetPosition().SquaredDistance(
          last_keyframe->GetSensorPose().GetPosition());
        if (oldmap_keyframe_distance_ <= 0.0 ||
          spacing_sq >= oldmap_keyframe_distance_ * oldmap_keyframe_distance_)
        {
          is_keyframe = true;
          last_keyframe = scan;
        }
      }

      if (!is_keyframe) {
        continue;
      }

      const kt_double current_distance_sq =
        scan->GetSensorPose().GetPosition().SquaredDistance(predicted_pose.GetPosition());
      if (current_distance_sq > radius_sq) {
        continue;
      }

      const std::size_t before = references.size();
      AddReference(scan, seen, references, raw_weights);
      if (references.size() != before) {
        historical_added++;
        if (historical_added >= oldmap_history_max_keyframes_) {
          break;
        }
      }
    }

    NormalizeRawWeights(raw_weights, normalized_weights);
  }

  karto::PointVectorDouble FindValidPointsWeighted(
    karto::LocalizedRangeScan * pScan,
    const karto::Vector2<kt_double> & viewPoint) const
  {
    const karto::PointVectorDouble & readings = pScan->GetPointReadings();
    const kt_double min_square_distance = karto::math::Square(0.1);

    karto::PointVectorDouble::const_iterator trailing = readings.begin();
    karto::PointVectorDouble valid_points;
    karto::Vector2<kt_double> first_point;
    kt_bool first_time = true;

    for (auto iter = readings.begin(); iter != readings.end(); ++iter) {
      karto::Vector2<kt_double> current_point = *iter;
      if (first_time && !std::isnan(current_point.GetX()) && !std::isnan(current_point.GetY())) {
        first_point = current_point;
        first_time = false;
      }

      if (first_time) {
        continue;
      }

      karto::Vector2<kt_double> delta = first_point - current_point;
      if (delta.SquaredLength() > min_square_distance) {
        const double a = viewPoint.GetY() - first_point.GetY();
        const double b = first_point.GetX() - viewPoint.GetX();
        const double c = first_point.GetY() * viewPoint.GetX() -
          first_point.GetX() * viewPoint.GetY();
        const double side = current_point.GetX() * a + current_point.GetY() * b + c;

        first_point = current_point;
        if (side < 0.0) {
          trailing = iter;
        } else {
          for (; trailing != iter; ++trailing) {
            valid_points.push_back(*trailing);
          }
        }
      }
    }

    return valid_points;
  }

  void AddWeightedReferenceScan(
    karto::CorrelationGrid * grid,
    karto::LocalizedRangeScan * scan,
    const karto::Vector2<kt_double> & view_point,
    kt_double confidence) const
  {
    const karto::PointVectorDouble valid_points =
      FindValidPointsWeighted(scan, view_point);
    const kt_double resolution = grid->GetResolution();
    const kt_double smear = m_pCorrelationSearchSpaceSmearDeviation->GetValue();
    const kt_int32s half_kernel = static_cast<kt_int32s>(
      karto::math::Round(2.0 * smear / resolution));

    for (const karto::Vector2<kt_double> & point : valid_points) {
      karto::Vector2<kt_int32s> grid_point = grid->WorldToGrid(point);
      if (!karto::math::IsUpTo(grid_point.GetX(), grid->GetROI().GetWidth()) ||
        !karto::math::IsUpTo(grid_point.GetY(), grid->GetROI().GetHeight()))
      {
        continue;
      }

      // Reproduce Karto's Gaussian smear kernel, but scale each scan's entire
      // kernel by its temporal confidence. Cell fusion remains max-based, so
      // contradictory older evidence dominates contradictory newer evidence.
      for (kt_int32s j = -half_kernel; j <= half_kernel; ++j) {
        kt_int8u * row = grid->GetDataPointer(
          karto::Vector2<kt_int32s>(grid_point.GetX(), grid_point.GetY() + j));
        for (kt_int32s i = -half_kernel; i <= half_kernel; ++i) {
          const kt_double distance = std::hypot(i * resolution, j * resolution);
          const kt_double z = std::exp(-0.5 * std::pow(distance / smear, 2));
          const kt_int32u weighted_value = static_cast<kt_int32u>(karto::math::Round(
            z * static_cast<kt_double>(karto::GridStates_Occupied) * confidence));
          if (weighted_value > row[i]) {
            row[i] = static_cast<kt_int8u>(weighted_value);
          }
        }
      }
    }
  }

  kt_double WeightedMatchScan(
    karto::LocalizedRangeScan * pScan,
    const karto::LocalizedRangeScanVector & references,
    const std::vector<kt_double> & weights,
    karto::Pose2 & mean,
    karto::Matrix3 & covariance,
    kt_bool do_penalize)
  {
    karto::Pose2 scan_pose = pScan->GetSensorPose();
    if (pScan->GetNumberOfRangeReadings() == 0) {
      mean = scan_pose;
      covariance(0, 0) = 500.0;
      covariance(1, 1) = 500.0;
      covariance(2, 2) = 4 * karto::math::Square(
        m_pCoarseAngleResolution->GetValue());
      return 0.0;
    }

    karto::CorrelationGrid * grid = m_pSequentialScanMatcher->GetCorrelationGrid();
    const karto::Rectangle2<kt_int32s> roi = grid->GetROI();

    karto::Vector2<kt_double> offset;
    offset.SetX(
      scan_pose.GetX() - 0.5 * (roi.GetWidth() - 1) * grid->GetResolution());
    offset.SetY(
      scan_pose.GetY() - 0.5 * (roi.GetHeight() - 1) * grid->GetResolution());
    grid->GetCoordinateConverter()->SetOffset(offset);
    grid->Clear();

    const karto::Vector2<kt_double> view_point = scan_pose.GetPosition();
    const std::size_t count = std::min(references.size(), weights.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (references[i] != nullptr) {
        AddWeightedReferenceScan(grid, references[i], view_point, weights[i]);
      }
    }

    // Match Karto's original search-space geometry. The correlation grid ROI
    // is intentionally much larger because it includes laser-range margins;
    // using ROI width here would incorrectly make the pose search enormous.
    const kt_double search_dimension =
      m_pCorrelationSearchSpaceDimension->GetValue();
    const kt_double search_resolution = grid->GetResolution();
    const kt_int32u search_side_cells = static_cast<kt_int32u>(
      karto::math::Round(search_dimension / search_resolution) + 1);
    const kt_double search_half_extent =
      0.5 * (static_cast<kt_double>(search_side_cells) - 1.0) * search_resolution;
    const karto::Vector2<kt_double> coarse_search_offset(
      search_half_extent, search_half_extent);
    const karto::Vector2<kt_double> coarse_search_resolution(
      2 * search_resolution, 2 * search_resolution);

    kt_double best_response = m_pSequentialScanMatcher->CorrelateScan(
      pScan,
      scan_pose,
      coarse_search_offset,
      coarse_search_resolution,
      m_pCoarseSearchAngleOffset->GetValue(),
      m_pCoarseAngleResolution->GetValue(),
      do_penalize,
      mean,
      covariance,
      false);

    if (m_pUseResponseExpansion->GetValue() &&
      karto::math::DoubleEqual(best_response, 0.0))
    {
      kt_double expanded_angle = m_pCoarseSearchAngleOffset->GetValue();
      for (kt_int32u i = 0; i < 3; ++i) {
        expanded_angle += karto::math::DegreesToRadians(20);
        best_response = m_pSequentialScanMatcher->CorrelateScan(
          pScan,
          scan_pose,
          coarse_search_offset,
          coarse_search_resolution,
          expanded_angle,
          m_pCoarseAngleResolution->GetValue(),
          do_penalize,
          mean,
          covariance,
          false);
        if (!karto::math::DoubleEqual(best_response, 0.0)) {
          break;
        }
      }
    }

    const karto::Vector2<kt_double> fine_search_offset(coarse_search_resolution * 0.5);
    const karto::Vector2<kt_double> fine_search_resolution(
      search_resolution, search_resolution);
    best_response = m_pSequentialScanMatcher->CorrelateScan(
      pScan,
      mean,
      fine_search_offset,
      fine_search_resolution,
      0.5 * m_pCoarseAngleResolution->GetValue(),
      m_pFineSearchAngleOffset->GetValue(),
      do_penalize,
      mean,
      covariance,
      true);

    return std::min<kt_double>(1.0, best_response);
  }

  karto::LocalizedRangeScan * GetClosestScanToPoseOldMap(
    const karto::LocalizedRangeScanVector & scans,
    const karto::Pose2 & pose) const
  {
    karto::LocalizedRangeScan * closest_scan = nullptr;
    kt_double best_squared_distance = DBL_MAX;

    for (karto::LocalizedRangeScan * scan : scans) {
      if (scan == nullptr) {
        continue;
      }
      const karto::Pose2 scan_pose = scan->GetReferencePose(
        m_pUseScanBarycenter->GetValue());
      const kt_double squared_distance =
        pose.GetPosition().SquaredDistance(scan_pose.GetPosition());
      if (squared_distance < best_squared_distance) {
        best_squared_distance = squared_distance;
        closest_scan = scan;
      }
    }

    return closest_scan;
  }

  void LinkScansOldMap(
    karto::LocalizedRangeScan * from_scan,
    karto::LocalizedRangeScan * to_scan,
    const karto::Pose2 & mean,
    const karto::Matrix3 & covariance)
  {
    if (from_scan == nullptr || to_scan == nullptr) {
      return;
    }

    kt_bool is_new_edge = true;
    karto::Edge<karto::LocalizedRangeScan> * edge =
      m_pGraph->AddEdge(from_scan, to_scan, is_new_edge);
    if (edge == nullptr || !is_new_edge) {
      return;
    }

    edge->SetLabel(new karto::LinkInfo(
      from_scan->GetCorrectedPose(),
      to_scan->GetCorrectedAt(mean),
      covariance));
    if (m_pScanOptimizer != nullptr) {
      m_pScanOptimizer->AddConstraint(edge);
    }
  }

  void LinkChainToScanOldMap(
    const karto::LocalizedRangeScanVector & chain,
    karto::LocalizedRangeScan * pScan,
    const karto::Pose2 & mean,
    const karto::Matrix3 & covariance)
  {
    if (chain.empty()) {
      return;
    }

    const karto::Pose2 pose = pScan->GetReferencePose(
      m_pUseScanBarycenter->GetValue());
    karto::LocalizedRangeScan * closest_scan =
      GetClosestScanToPoseOldMap(chain, pose);
    if (closest_scan == nullptr) {
      return;
    }

    const karto::Pose2 closest_pose = closest_scan->GetReferencePose(
      m_pUseScanBarycenter->GetValue());
    const kt_double squared_distance =
      pose.GetPosition().SquaredDistance(closest_pose.GetPosition());
    if (squared_distance <
      karto::math::Square(m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
    {
      LinkScansOldMap(closest_scan, pScan, mean, covariance);
    }
  }

  std::vector<karto::LocalizedRangeScanVector> FindNearChainsOldMap(
    karto::LocalizedRangeScan * pScan)
  {
    std::vector<karto::LocalizedRangeScanVector> near_chains;
    const karto::Pose2 scan_pose = pScan->GetReferencePose(
      m_pUseScanBarycenter->GetValue());

    karto::LocalizedRangeScanVector processed;
    const karto::LocalizedRangeScanVector near_linked_scans =
      m_pGraph->FindNearLinkedScans(
        pScan, m_pLinkScanMaximumDistance->GetValue());

    for (karto::LocalizedRangeScan * near_scan : near_linked_scans) {
      if (near_scan == nullptr || near_scan == pScan) {
        continue;
      }
      if (std::find(processed.begin(), processed.end(), near_scan) != processed.end()) {
        continue;
      }

      processed.push_back(near_scan);
      bool is_valid_chain = true;
      std::list<karto::LocalizedRangeScan *> chain;

      for (kt_int32s candidate_num = near_scan->GetStateId() - 1;
        candidate_num >= 0; --candidate_num)
      {
        karto::LocalizedRangeScan * candidate =
          m_pMapperSensorManager->GetScan(near_scan->GetSensorName(), candidate_num);
        if (candidate == pScan) {
          is_valid_chain = false;
        }
        if (candidate == nullptr) {
          continue;
        }

        const karto::Pose2 candidate_pose = candidate->GetReferencePose(
          m_pUseScanBarycenter->GetValue());
        const kt_double squared_distance =
          scan_pose.GetPosition().SquaredDistance(candidate_pose.GetPosition());
        if (squared_distance <
          karto::math::Square(m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_front(candidate);
          processed.push_back(candidate);
        } else {
          break;
        }
      }

      chain.push_back(near_scan);

      const kt_int32u end = static_cast<kt_int32u>(
        m_pMapperSensorManager->GetScans(near_scan->GetSensorName()).size());
      for (kt_int32u candidate_num =
          static_cast<kt_int32u>(near_scan->GetStateId() + 1);
        candidate_num < end; ++candidate_num)
      {
        karto::LocalizedRangeScan * candidate =
          m_pMapperSensorManager->GetScan(near_scan->GetSensorName(), candidate_num);
        if (candidate == pScan) {
          is_valid_chain = false;
        }
        if (candidate == nullptr) {
          continue;
        }

        const karto::Pose2 candidate_pose = candidate->GetReferencePose(
          m_pUseScanBarycenter->GetValue());
        const kt_double squared_distance =
          scan_pose.GetPosition().SquaredDistance(candidate_pose.GetPosition());
        if (squared_distance <
          karto::math::Square(m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_back(candidate);
          processed.push_back(candidate);
        } else {
          break;
        }
      }

      if (is_valid_chain) {
        karto::LocalizedRangeScanVector temp_chain;
        std::copy(chain.begin(), chain.end(), std::back_inserter(temp_chain));
        near_chains.push_back(temp_chain);
      }
    }

    return near_chains;
  }

  karto::Pose2 ComputeWeightedMeanOldMap(
    const karto::Pose2Vector & means,
    const std::vector<karto::Matrix3> & covariances) const
  {
    assert(means.size() == covariances.size());

    std::vector<karto::Matrix3> inverses;
    inverses.reserve(covariances.size());
    karto::Matrix3 sum_of_inverses;
    for (const karto::Matrix3 & covariance : covariances) {
      karto::Matrix3 inverse = covariance.Inverse();
      inverses.push_back(inverse);
      sum_of_inverses += inverse;
    }
    const karto::Matrix3 inverse_of_sum = sum_of_inverses.Inverse();

    karto::Pose2 accumulated_pose;
    kt_double theta_x = 0.0;
    kt_double theta_y = 0.0;
    for (std::size_t i = 0; i < means.size(); ++i) {
      const karto::Pose2 pose = means[i];
      const kt_double angle = pose.GetHeading();
      theta_x += std::cos(angle);
      theta_y += std::sin(angle);

      const karto::Matrix3 weight = inverse_of_sum * inverses[i];
      accumulated_pose += weight * pose;
    }

    theta_x /= means.size();
    theta_y /= means.size();
    accumulated_pose.SetHeading(std::atan2(theta_y, theta_x));
    return accumulated_pose;
  }

  void LinkNearChainsOldMap(
    karto::LocalizedRangeScan * pScan,
    karto::Pose2Vector & means,
    std::vector<karto::Matrix3> & covariances)
  {
    const std::vector<karto::LocalizedRangeScanVector> near_chains =
      FindNearChainsOldMap(pScan);

    for (const karto::LocalizedRangeScanVector & chain : near_chains) {
      if (chain.size() < m_pLoopMatchMinimumChainSize->GetValue()) {
        continue;
      }

      std::vector<kt_double> weights;
      BuildWeightsForReferences(chain, weights);

      karto::Pose2 mean;
      karto::Matrix3 covariance;
      const kt_double response = WeightedMatchScan(
        pScan, chain, weights, mean, covariance, false);

      if (response > m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE) {
        means.push_back(mean);
        covariances.push_back(covariance);
        LinkChainToScanOldMap(chain, pScan, mean, covariance);
      }
    }
  }

  void AddEdgesOldMap(
    karto::LocalizedRangeScan * pScan,
    const karto::Matrix3 & covariance)
  {
    karto::MapperSensorManager * sensor_manager = m_pMapperSensorManager;
    const karto::Name sensor_name = pScan->GetSensorName();

    // Same first edge as Karto: always link the accepted scan to the previous
    // scan for this sensor.
    const kt_int32s previous_scan_num = pScan->GetStateId() - 1;
    if (sensor_manager->GetLastScan(sensor_name) != nullptr) {
      assert(previous_scan_num >= 0);
      karto::LocalizedRangeScan * previous_scan =
        sensor_manager->GetScan(sensor_name, previous_scan_num);
      if (previous_scan == nullptr) {
        return;
      }
      LinkScansOldMap(previous_scan, pScan, pScan->GetSensorPose(), covariance);
    }

    karto::Pose2Vector means;
    std::vector<karto::Matrix3> covariances;

    if (sensor_manager->GetLastScan(sensor_name) == nullptr) {
      // Preserve upstream multi-sensor initialization behavior. This project
      // normally has one laser, but keeping this branch avoids changing Karto
      // semantics unnecessarily.
      assert(sensor_manager->GetScans(sensor_name).size() == 1);
      const std::vector<karto::Name> device_names = sensor_manager->GetSensorNames();
      for (const karto::Name & candidate_sensor_name : device_names) {
        if (candidate_sensor_name == sensor_name ||
          sensor_manager->GetScans(candidate_sensor_name).empty())
        {
          continue;
        }

        karto::Pose2 best_pose;
        karto::Matrix3 candidate_covariance;
        const kt_double response =
          m_pSequentialScanMatcher->MatchScan<karto::LocalizedRangeScanMap>(
            pScan,
            sensor_manager->GetScans(candidate_sensor_name),
            best_pose,
            candidate_covariance);

        LinkScansOldMap(
          sensor_manager->GetScan(candidate_sensor_name, 0),
          pScan,
          best_pose,
          candidate_covariance);

        if (response > m_pLinkMatchMinimumResponseFine->GetValue()) {
          means.push_back(best_pose);
          covariances.push_back(candidate_covariance);
        }
      }
    } else {
      const karto::Pose2 scan_pose = pScan->GetSensorPose();
      means.push_back(scan_pose);
      covariances.push_back(covariance);
      LinkChainToScanOldMap(
        sensor_manager->GetRunningScans(sensor_name),
        pScan,
        scan_pose,
        covariance);
    }

    // Intentional V2 change: near-chain local matching now uses temporal
    // old>new confidence instead of Karto's unweighted MatchScan().
    LinkNearChainsOldMap(pScan, means, covariances);

    if (!means.empty()) {
      pScan->SetSensorPose(ComputeWeightedMeanOldMap(means, covariances));
    }
  }

  bool oldmap_scan_weighting_enabled_;
  double oldmap_scan_min_confidence_;
  double oldmap_scan_decay_nodes_;
  bool oldmap_keep_first_scan_;
  double oldmap_keyframe_distance_;
  double oldmap_history_search_radius_;
  int oldmap_history_max_keyframes_;
};

}  // namespace mapper_utils

#endif  // SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_
