/*
 * Lightweight old-map-first scan refinement for slam_toolbox.
 *
 * The expensive weighted correlation grid used by the first prototype is not
 * used in this path.  When enabled, each accepted scan is handled in two
 * stages:
 *
 *   1. Karto's normal sequential matcher aligns the new scan against the
 *      ordinary recent running buffer.
 *   2. At most a few sparse, genuinely historical keyframes near that stock
 *      pose are matched with the same upstream Karto matcher.  A confidence-
 *      gated and tightly capped fraction of that historical correction is then
 *      applied to the stock pose.
 *
 * This preserves normal local continuity and gives old observations a small
 * scan-level influence without rebuilding a custom Gaussian grid for all of
 * the recent scans on every update.  Graph construction and loop closure stay
 * identical to upstream Karto.
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

  kt_bool Process(
    karto::LocalizedRangeScan * pScan,
    karto::Matrix3 * covariance = nullptr) override
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
      const karto::LocalizedRangeScanVector & running =
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName());

      // Stage 1: keep the normal Karto local matcher unchanged for the entire
      // recent running buffer.  This is the pose that remains authoritative.
      karto::Pose2 recent_pose;
      m_pSequentialScanMatcher->MatchScan(
        pScan,
        running,
        recent_pose,
        cov);
      pScan->SetSensorPose(recent_pose);

      // Stage 2: build a tiny historical-only reference set around the stock
      // pose.  Scans still present in the running buffer are explicitly
      // excluded, so they are never rasterized a second time here.
      karto::LocalizedRangeScanVector historical;
      std::vector<kt_double> historical_confidences;
      BuildHistoricalReferenceSet(
        pScan,
        running,
        historical,
        historical_confidences);

      if (!historical.empty()) {
        karto::Pose2 historical_pose;
        karto::Matrix3 historical_cov;
        historical_cov.SetToIdentity();

        // Use upstream Karto again.  Skip the optional fine pass to keep this
        // second, old-map-only correction lightweight.
        const kt_double historical_response =
          m_pSequentialScanMatcher->MatchScan(
          pScan,
          historical,
          historical_pose,
          historical_cov,
          true,
          false);

        // Ignore weak historical matches.  Strong matches can only nudge the
        // stock pose by a small bounded amount; they can never replace it.
        if (historical_response >= 0.55) {
          const karto::Pose2 refined_pose = BlendHistoricalCorrection(
            recent_pose,
            historical_pose,
            historical_confidences,
            historical_response);
          pScan->SetSensorPose(refined_pose);
        } else {
          pScan->SetSensorPose(recent_pose);
        }
      }

      // Keep covariance from the normal recent-buffer match.  The historical
      // pass is deliberately only a small prior-like pose refinement.
      if (covariance != nullptr) {
        *covariance = cov;
      }
    }

    // The remainder mirrors upstream Mapper::Process().
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);
      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<karto::Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
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

  void BuildHistoricalReferenceSet(
    karto::LocalizedRangeScan * pScan,
    const karto::LocalizedRangeScanVector & running,
    karto::LocalizedRangeScanVector & historical,
    std::vector<kt_double> & confidences) const
  {
    historical.clear();
    confidences.clear();

    const std::set<karto::LocalizedRangeScan *> running_set(
      running.begin(), running.end());
    const karto::Pose2 predicted_pose = pScan->GetSensorPose();
    const kt_double radius_sq =
      oldmap_history_search_radius_ * oldmap_history_search_radius_;

    karto::LocalizedRangeScanMap & all_scans =
      m_pMapperSensorManager->GetScans(pScan->GetSensorName());

    karto::LocalizedRangeScan * last_keyframe = nullptr;

    // Iteration is chronological.  When more candidates are available than
    // the cap permits, the oldest spatially relevant keyframes win.
    for (auto & item : all_scans) {
      karto::LocalizedRangeScan * scan = item.second;
      if (scan == nullptr) {
        continue;
      }

      bool is_keyframe = false;
      if (scan->GetStateId() == 0) {
        is_keyframe = oldmap_keep_first_scan_;
        // Keep scan zero as the spacing origin even if direct use is disabled.
        last_keyframe = scan;
      } else if (last_keyframe == nullptr) {
        is_keyframe = true;
        last_keyframe = scan;
      } else {
        const kt_double spacing_sq =
          scan->GetSensorPose().GetPosition().SquaredDistance(
          last_keyframe->GetSensorPose().GetPosition());
        if (oldmap_keyframe_distance_ <= 0.0 ||
          spacing_sq >=
          oldmap_keyframe_distance_ * oldmap_keyframe_distance_)
        {
          is_keyframe = true;
          last_keyframe = scan;
        }
      }

      if (!is_keyframe) {
        continue;
      }

      // A scan is historical here only after it has left the normal recent
      // running buffer.  This is the key difference from the heavy prototype.
      if (running_set.find(scan) != running_set.end()) {
        continue;
      }

      const kt_double distance_sq =
        scan->GetSensorPose().GetPosition().SquaredDistance(
        predicted_pose.GetPosition());
      if (distance_sq > radius_sq) {
        continue;
      }

      historical.push_back(scan);
      confidences.push_back(RawConfidence(scan));
      if (static_cast<int>(historical.size()) >=
        oldmap_history_max_keyframes_)
      {
        break;
      }
    }
  }

  karto::Pose2 BlendHistoricalCorrection(
    const karto::Pose2 & recent_pose,
    const karto::Pose2 & historical_pose,
    const std::vector<kt_double> & confidences,
    kt_double response) const
  {
    // Bound the raw old-map suggestion before blending.  With the maximum
    // gain below, the actual per-scan correction is <= 9 cm and <= 2.25 deg.
    constexpr kt_double kRawTranslationCapM = 0.20;
    const kt_double raw_yaw_cap = karto::math::DegreesToRadians(5.0);

    kt_double dx = historical_pose.GetX() - recent_pose.GetX();
    kt_double dy = historical_pose.GetY() - recent_pose.GetY();
    const kt_double distance = std::hypot(dx, dy);
    if (distance > kRawTranslationCapM && distance > 1e-9) {
      const kt_double scale = kRawTranslationCapM / distance;
      dx *= scale;
      dy *= scale;
    }

    kt_double dyaw = std::atan2(
      std::sin(historical_pose.GetHeading() - recent_pose.GetHeading()),
      std::cos(historical_pose.GetHeading() - recent_pose.GetHeading()));
    dyaw = std::max(-raw_yaw_cap, std::min(raw_yaw_cap, dyaw));

    kt_double confidence = oldmap_scan_min_confidence_;
    if (!confidences.empty()) {
      kt_double sum = 0.0;
      for (kt_double value : confidences) {
        sum += value;
      }
      confidence = sum / static_cast<kt_double>(confidences.size());
    }

    // Old history remains influential, but recent scan matching stays dominant.
    const kt_double gain = std::max<kt_double>(
      0.0,
      std::min<kt_double>(0.45, 0.45 * confidence * response));

    const kt_double refined_heading = std::atan2(
      std::sin(recent_pose.GetHeading() + gain * dyaw),
      std::cos(recent_pose.GetHeading() + gain * dyaw));

    return karto::Pose2(
      recent_pose.GetX() + gain * dx,
      recent_pose.GetY() + gain * dy,
      refined_heading);
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
