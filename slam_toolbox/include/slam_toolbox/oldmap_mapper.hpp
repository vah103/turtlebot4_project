/*
 * Lightweight old-map scan influence for slam_toolbox.
 *
 * This implementation deliberately keeps karto::Mapper::Process() authoritative.
 * OldMapMapper does not duplicate the stock processing pipeline. Instead, it
 * temporarily prepends a very small number of cached historical keyframes to
 * Karto's ordinary recent running-scan buffer, calls the stock Process() once,
 * then removes those temporary references again.
 *
 * Consequences:
 *   - exactly one stock sequential MatchScan() is executed per accepted scan;
 *   - graph construction, running-buffer maintenance and loop closure are the
 *     upstream Karto implementations;
 *   - no second historical MatchScan() is executed;
 *   - no custom weighted/Gaussian correlation grid is built;
 *   - no full all_scans traversal is performed on every scan.
 *
 * Historical keyframes are cached incrementally and only become eligible once
 * they have left the normal recent running buffer. With the current launch
 * profile, at most three spatially nearby old keyframes are added to the one
 * stock scan-match reference set.
 */

#ifndef SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_
#define SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_

#include <algorithm>
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
    oldmap_history_max_keyframes_(40),
    last_cached_keyframe_(nullptr)
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
    if (!oldmap_scan_weighting_enabled_ || pScan == nullptr) {
      return karto::Mapper::Process(pScan, covariance);
    }

    const karto::Name sensor_name = pScan->GetSensorName();

    // GetLastScan() also registers a new sensor in Karto when needed, so it
    // must be queried before asking for that sensor's running buffer.
    karto::LocalizedRangeScan * last_scan =
      m_pMapperSensorManager->GetLastScan(sensor_name);
    karto::LocalizedRangeScanVector & running =
      m_pMapperSensorManager->GetRunningScans(sensor_name);

    if (last_scan == nullptr) {
      historical_keyframes_.clear();
      last_cached_keyframe_ = nullptr;
    }

    karto::LocalizedRangeScanVector injected_history;
    if (last_scan != nullptr && !historical_keyframes_.empty()) {
      // Reproduce only the stock odometry->corrected pose prediction for
      // candidate selection, without modifying the scan. Mapper::Process()
      // remains responsible for the real pose update and all processing.
      karto::Transform last_transform(
        last_scan->GetOdometricPose(), last_scan->GetCorrectedPose());
      const karto::Pose2 predicted_pose =
        last_transform.TransformPose(pScan->GetOdometricPose());

      SelectNearbyHistorical(predicted_pose, running, injected_history);

      // Prepend temporary old references. Karto's MatchScan is occupancy-grid
      // based, so reference order does not change the correlation semantics.
      // Prepending also means Karto's normal size trimming removes temporary
      // references before genuine recent scans if the buffer is already full.
      running.insert(
        running.begin(),
        injected_history.begin(),
        injected_history.end());
    }

    // Crucial point: the complete stock Karto processing pipeline runs here.
    const kt_bool accepted = karto::Mapper::Process(pScan, covariance);

    // Remove any temporary history references that survived Karto's normal
    // running-buffer trimming. The persistent recent buffer therefore remains
    // a normal stock-style recent buffer for the next scan.
    if (!injected_history.empty()) {
      running.erase(
        std::remove_if(
          running.begin(),
          running.end(),
          [&injected_history](karto::LocalizedRangeScan * scan) {
            return std::find(
              injected_history.begin(),
              injected_history.end(),
              scan) != injected_history.end();
          }),
        running.end());
    }

    if (accepted) {
      RememberKeyframe(pScan);
    }

    return accepted;
  }

private:
  void SelectNearbyHistorical(
    const karto::Pose2 & predicted_pose,
    const karto::LocalizedRangeScanVector & running,
    karto::LocalizedRangeScanVector & selected) const
  {
    selected.clear();
    const kt_double radius_sq =
      oldmap_history_search_radius_ * oldmap_history_search_radius_;

    for (karto::LocalizedRangeScan * scan : historical_keyframes_) {
      if (scan == nullptr) {
        continue;
      }

      // Only truly historical scans are eligible. Recent scans are already in
      // the stock buffer and must not be duplicated in the same match.
      if (std::find(running.begin(), running.end(), scan) != running.end()) {
        continue;
      }

      const kt_double distance_sq =
        scan->GetSensorPose().GetPosition().SquaredDistance(
          predicted_pose.GetPosition());
      if (distance_sq > radius_sq) {
        continue;
      }

      selected.push_back(scan);
      if (static_cast<int>(selected.size()) >= oldmap_history_max_keyframes_) {
        break;
      }
    }
  }

  void RememberKeyframe(karto::LocalizedRangeScan * scan)
  {
    if (scan == nullptr) {
      return;
    }

    // State zero marks a fresh scan sequence. This also makes the cache robust
    // to a mapper reset even if Process() was not called while last_scan=null.
    if (scan->GetStateId() == 0) {
      historical_keyframes_.clear();
      last_cached_keyframe_ = scan;
      if (oldmap_keep_first_scan_) {
        historical_keyframes_.push_back(scan);
      }
      return;
    }

    if (last_cached_keyframe_ == nullptr) {
      last_cached_keyframe_ = scan;
      historical_keyframes_.push_back(scan);
      return;
    }

    const kt_double spacing_sq =
      scan->GetSensorPose().GetPosition().SquaredDistance(
        last_cached_keyframe_->GetSensorPose().GetPosition());
    if (oldmap_keyframe_distance_ <= 0.0 ||
      spacing_sq >= oldmap_keyframe_distance_ * oldmap_keyframe_distance_)
    {
      historical_keyframes_.push_back(scan);
      last_cached_keyframe_ = scan;
    }
  }

  bool oldmap_scan_weighting_enabled_;
  double oldmap_scan_min_confidence_;
  double oldmap_scan_decay_nodes_;
  bool oldmap_keep_first_scan_;
  double oldmap_keyframe_distance_;
  double oldmap_history_search_radius_;
  int oldmap_history_max_keyframes_;

  karto::LocalizedRangeScanVector historical_keyframes_;
  karto::LocalizedRangeScan * last_cached_keyframe_;
};

}  // namespace mapper_utils

#endif  // SLAM_TOOLBOX__OLDMAP_MAPPER_HPP_
