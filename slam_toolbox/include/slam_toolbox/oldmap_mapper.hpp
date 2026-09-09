/*
 * Old-map-first scan-matching diagnostic for slam_toolbox.
 *
 * This mapper keeps upstream Karto graph construction and loop closure intact,
 * but changes the sequential/local scan-matching reference grid when explicitly
 * enabled. Earlier scans contribute more strongly than later scans, while the
 * first pose remains hard-fixed by the existing Ceres solver behavior.
 */

#ifndef SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_
#define SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_

#include <algorithm>
#include <cmath>
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

    if (m_pUseScanMatching->GetValue() && pLastScan != nullptr) {
      karto::Pose2 bestPose;
      karto::LocalizedRangeScanVector references;
      std::vector<kt_double> weights;
      BuildReferenceSet(pScan, references, weights);

      if (!references.empty()) {
        WeightedMatchScan(pScan, references, weights, bestPose, cov);
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

    // The rest is intentionally identical to upstream Mapper::Process().
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);
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

    if (references.empty()) {
      return;
    }

    // Use global age confidence for relative ordering, then normalize by the
    // strongest active reference. This keeps S1>S2>S3... when old history is
    // present without collapsing the absolute scan-matcher response when the
    // robot is exploring only a new region.
    const kt_double max_raw = *std::max_element(raw_weights.begin(), raw_weights.end());
    const kt_double denominator = std::max<kt_double>(max_raw, 1e-6);
    normalized_weights.reserve(raw_weights.size());
    for (kt_double raw : raw_weights) {
      normalized_weights.push_back(
        std::max<kt_double>(0.01, std::min<kt_double>(1.0, raw / denominator)));
    }
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
    karto::Matrix3 & covariance)
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
      true,
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
          true,
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
      true,
      mean,
      covariance,
      true);

    return std::min<kt_double>(1.0, best_response);
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
