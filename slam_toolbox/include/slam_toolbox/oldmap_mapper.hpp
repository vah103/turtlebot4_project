/*
 * Old-map historical-search A/B diagnostic for slam_toolbox.
 *
 * This diagnostic keeps the same lightweight OldMap path and historical
 * candidate search as the previous two-pass prototype, but deliberately
 * disables only the second historical MatchScan() call.
 *
 * Each accepted scan is therefore handled as follows:
 *
 *   1. Karto's normal sequential matcher aligns the new scan against the
 *      ordinary recent running buffer.
 *   2. Up to a few sparse, genuinely historical keyframes near that stock pose
 *      are still searched and collected exactly as before.
 *   3. No second scan matching / CorrelateScan pass is executed on those
 *      historical keyframes; the stock recent-buffer pose remains final.
 *
 * This isolates the CPU/navigation impact of the second historical scan-match
 * pass from the cost of merely searching the historical scan set. Graph
 * construction and loop closure stay identical to upstream Karto.
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
      // recent running buffer. This stock result remains authoritative.
      karto::Pose2 recent_pose;
      m_pSequentialScanMatcher->MatchScan(
        pScan,
        running,
        recent_pose,
        cov);
      pScan->SetSensorPose(recent_pose);

      // Stage 2 diagnostic: perform exactly the same historical candidate
      // search as the two-pass prototype, including running-buffer exclusion,
      // keyframe spacing, radius gating and the configured maximum count.
      // Deliberately do NOT call MatchScan() on this set. This lets the runtime
      // test isolate historical-search overhead from CorrelateScan overhead.
      karto::LocalizedRangeScanVector historical;
      std::vector<kt_double> historical_confidences;
      BuildHistoricalReferenceSet(
        pScan,
        running,
        historical,
        historical_confidences);

      // No historical pose correction is applied in this A/B branch.
      pScan->SetSensorPose(recent_pose);

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

    // Iteration is chronological. When more candidates are available than
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
      // running buffer, exactly as in the preceding two-pass prototype.
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
