/*
 * Old-map custom-Process A/B diagnostic for slam_toolbox.
 *
 * Test C deliberately removes all historical-scan work while keeping the
 * OldMapMapper override active. With oldmap_scan_weighting_enabled=true,
 * accepted scans still execute this custom Process() implementation instead of
 * delegating to karto::Mapper::Process(). The processing path below mirrors
 * upstream sequential scan matching, graph construction and loop closure only.
 *
 * Historical scan lookup, historical MatchScan(), weighted correlation and
 * historical pose correction are all disabled in this diagnostic.
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
      m_pSequentialScanMatcher->MatchScan(
        pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);

      if (covariance != nullptr) {
        *covariance = cov;
      }
    }

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
