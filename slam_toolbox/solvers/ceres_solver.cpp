/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>
#include "ceres_solver.hpp"

namespace solver_plugins
{

namespace
{

struct AdaptiveEdgeRecord
{
  int node1;
  int node2;
  Eigen::Vector3d measurement;
  Eigen::Matrix3d base_sqrt_information;
  ceres::ResidualBlockId block;
  bool temporal_local;
  bool loop_candidate;
  double temporal_weight;
  double effective_weight;
};

struct AdaptiveReleaseRegion
{
  int start_node;
  int end_node;
  double factor;
};

struct AdaptiveLoopObservation
{
  int older_node;
  int newer_node;
  Eigen::Vector2d requested_global_translation;
  double requested_yaw;
  double mahalanobis_sq;
};

struct AdaptiveLoopEvidence
{
  bool valid = false;
  int region_start = 0;
  int region_end = 0;
  std::size_t edge_count = 0;
  double mean_mahalanobis_sq = 0.0;
};

// Adaptive Temporal Anchor V1.
//
// V1 deliberately leaves the upstream scan matcher, Karto graph construction,
// loop closure and whole-graph Ceres optimization intact. Only strict
// sequential edges receive an age-dependent information boost. Long-gap edges
// are never strengthened; they are used only as conservative loop-evidence
// candidates because LinkInfo does not expose an explicit edge-origin type to
// the ScanSolver API.
bool g_adaptive_anchor_enabled = false;
double g_adaptive_min_weight = 1.0;
double g_adaptive_max_weight = 3.0;
double g_adaptive_decay_nodes = 50.0;
int g_adaptive_local_edge_max_gap = 1;
int g_adaptive_loop_min_node_gap = 30;
int g_adaptive_loop_evidence_min_edges = 3;
int g_adaptive_loop_evidence_window_nodes = 120;
int g_adaptive_loop_evidence_min_new_node_separation = 2;
double g_adaptive_loop_consistency_translation_m = 0.20;
double g_adaptive_loop_consistency_yaw_rad = 0.05235987755982989;  // 3 deg
double g_adaptive_loop_min_mahalanobis_sq = 9.0;
double g_adaptive_release_stage1_factor = 0.5;
double g_adaptive_release_stage2_factor = 0.0;
int g_adaptive_first_node_id = -1;
int g_adaptive_last_summary_node = -1;

std::map<std::pair<int, int>, AdaptiveEdgeRecord> g_adaptive_edges;
std::vector<AdaptiveReleaseRegion> g_adaptive_release_regions;

std::pair<int, int> AdaptiveEdgeKey(const int node1, const int node2)
{
  return std::make_pair(std::min(node1, node2), std::max(node1, node2));
}

double AdaptiveNormalizeAngle(double angle)
{
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double AdaptiveTemporalWeight(const int older_node)
{
  const int first_node =
    g_adaptive_first_node_id >= 0 ? g_adaptive_first_node_id : older_node;
  const double age = static_cast<double>(std::max(0, older_node - first_node));
  const double decay = std::max(1.0, g_adaptive_decay_nodes);
  const double alpha = std::exp(-age / decay);
  return g_adaptive_min_weight +
         (g_adaptive_max_weight - g_adaptive_min_weight) * alpha;
}

double AdaptiveReleaseFactorForNode(const int older_node)
{
  double factor = 1.0;
  for (const AdaptiveReleaseRegion & region : g_adaptive_release_regions) {
    if (older_node >= region.start_node && older_node < region.end_node) {
      factor = std::min(factor, region.factor);
    }
  }
  return factor;
}

double AdaptiveEffectiveWeight(const AdaptiveEdgeRecord & edge)
{
  if (!g_adaptive_anchor_enabled || !edge.temporal_local) {
    return 1.0;
  }

  const int older_node = std::min(edge.node1, edge.node2);
  const double release = AdaptiveReleaseFactorForNode(older_node);
  return 1.0 + release * (edge.temporal_weight - 1.0);
}

bool AdaptiveRegionNeedsRelease(
  const int start_node, const int end_node, const double target_factor)
{
  for (const auto & item : g_adaptive_edges) {
    const AdaptiveEdgeRecord & edge = item.second;
    if (!edge.temporal_local) {
      continue;
    }

    const int older_node = std::min(edge.node1, edge.node2);
    if (older_node >= start_node && older_node < end_node &&
      AdaptiveReleaseFactorForNode(older_node) > target_factor + 1e-9)
    {
      return true;
    }
  }
  return false;
}

void AdaptiveAddReleaseRegion(
  const int start_node, const int end_node, const double factor)
{
  if (end_node <= start_node) {
    return;
  }

  AdaptiveReleaseRegion region;
  region.start_node = start_node;
  region.end_node = end_node;
  region.factor = std::max(0.0, std::min(1.0, factor));
  g_adaptive_release_regions.push_back(region);
}

bool AdaptiveRegionsOverlap(
  const AdaptiveLoopEvidence & a, const AdaptiveLoopEvidence & b)
{
  return a.valid && b.valid &&
         std::max(a.region_start, b.region_start) <
         std::min(a.region_end, b.region_end);
}

int AdaptiveLatestNodeId(const std::unordered_map<int, Eigen::Vector3d> & nodes)
{
  int latest = std::numeric_limits<int>::min();
  for (const auto & item : nodes) {
    latest = std::max(latest, item.first);
  }
  return latest;
}

AdaptiveLoopEvidence AdaptiveDetectStrongLoopEvidence(
  const std::unordered_map<int, Eigen::Vector3d> & nodes)
{
  AdaptiveLoopEvidence evidence;
  if (!g_adaptive_anchor_enabled ||
    static_cast<int>(g_adaptive_edges.size()) < g_adaptive_loop_evidence_min_edges)
  {
    return evidence;
  }

  const int latest_graph_node = AdaptiveLatestNodeId(nodes);
  if (latest_graph_node == std::numeric_limits<int>::min()) {
    return evidence;
  }

  std::vector<AdaptiveLoopObservation> candidates;
  for (const auto & item : g_adaptive_edges) {
    const AdaptiveEdgeRecord & edge = item.second;
    if (!edge.loop_candidate) {
      continue;
    }

    const int older_node = std::min(edge.node1, edge.node2);
    const int newer_node = std::max(edge.node1, edge.node2);
    if (latest_graph_node - newer_node > g_adaptive_loop_evidence_window_nodes) {
      continue;
    }

    const auto node1it = nodes.find(edge.node1);
    const auto node2it = nodes.find(edge.node2);
    if (node1it == nodes.end() || node2it == nodes.end()) {
      continue;
    }

    const double yaw1 = node1it->second(2);
    const double cos_yaw = std::cos(yaw1);
    const double sin_yaw = std::sin(yaw1);
    Eigen::Matrix2d rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

    const Eigen::Vector2d p1(node1it->second(0), node1it->second(1));
    const Eigen::Vector2d p2(node2it->second(0), node2it->second(1));
    const Eigen::Vector2d predicted_relative =
      rotation.transpose() * (p2 - p1);

    Eigen::Vector3d raw_error;
    raw_error.head<2>() = predicted_relative - edge.measurement.head<2>();
    raw_error(2) = AdaptiveNormalizeAngle(
      (node2it->second(2) - node1it->second(2)) - edge.measurement(2));

    // This is the unmodified Karto information matrix. Temporal weight is not
    // included here, so loop evidence is evaluated in the constraint's own
    // normalized residual space rather than against our added prior.
    const Eigen::Vector3d normalized_error =
      edge.base_sqrt_information * raw_error;
    const double mahalanobis_sq = normalized_error.squaredNorm();
    if (mahalanobis_sq < g_adaptive_loop_min_mahalanobis_sq) {
      continue;
    }

    AdaptiveLoopObservation observation;
    observation.older_node = older_node;
    observation.newer_node = newer_node;
    observation.requested_global_translation =
      rotation * (-raw_error.head<2>());
    observation.requested_yaw = AdaptiveNormalizeAngle(-raw_error(2));
    observation.mahalanobis_sq = mahalanobis_sq;
    candidates.push_back(observation);
  }

  if (static_cast<int>(candidates.size()) < g_adaptive_loop_evidence_min_edges) {
    return evidence;
  }

  // Newest-first lets one newly accepted scan contribute at most one vote when
  // several long-gap edges happen to be attached to that same scan.
  std::sort(
    candidates.begin(), candidates.end(),
    [](const AdaptiveLoopObservation & a, const AdaptiveLoopObservation & b) {
      return a.newer_node > b.newer_node;
    });

  std::vector<AdaptiveLoopObservation> best_cluster;
  for (const AdaptiveLoopObservation & seed : candidates) {
    std::vector<AdaptiveLoopObservation> cluster;
    for (const AdaptiveLoopObservation & candidate : candidates) {
      if ((candidate.requested_global_translation -
        seed.requested_global_translation).norm() >
        g_adaptive_loop_consistency_translation_m)
      {
        continue;
      }

      if (std::abs(AdaptiveNormalizeAngle(
        candidate.requested_yaw - seed.requested_yaw)) >
        g_adaptive_loop_consistency_yaw_rad)
      {
        continue;
      }

      bool independent = true;
      for (const AdaptiveLoopObservation & selected : cluster) {
        if (std::abs(candidate.newer_node - selected.newer_node) <
          g_adaptive_loop_evidence_min_new_node_separation)
        {
          independent = false;
          break;
        }
      }

      if (independent) {
        cluster.push_back(candidate);
      }
    }

    if (cluster.size() > best_cluster.size()) {
      best_cluster = cluster;
    }
  }

  if (static_cast<int>(best_cluster.size()) < g_adaptive_loop_evidence_min_edges) {
    return evidence;
  }

  evidence.valid = true;
  evidence.region_start = std::numeric_limits<int>::max();
  evidence.region_end = std::numeric_limits<int>::min();
  double mahalanobis_sum = 0.0;
  for (const AdaptiveLoopObservation & observation : best_cluster) {
    evidence.region_start = std::min(evidence.region_start, observation.older_node);
    evidence.region_end = std::max(evidence.region_end, observation.newer_node);
    mahalanobis_sum += observation.mahalanobis_sq;
  }
  evidence.edge_count = best_cluster.size();
  evidence.mean_mahalanobis_sq =
    mahalanobis_sum / static_cast<double>(best_cluster.size());
  return evidence;
}

void AdaptiveEraseConstraintMetadata(const int source_id, const int target_id)
{
  g_adaptive_edges.erase(AdaptiveEdgeKey(source_id, target_id));
}

void AdaptiveEraseNodeMetadata(const int node_id)
{
  for (auto it = g_adaptive_edges.begin(); it != g_adaptive_edges.end();) {
    if (it->second.node1 == node_id || it->second.node2 == node_id) {
      it = g_adaptive_edges.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace

/*****************************************************************************/
CeresSolver::CeresSolver()
: nodes_(new std::unordered_map<int, Eigen::Vector3d>()),
  blocks_(new std::unordered_map<std::size_t,
    ceres::ResidualBlockId>()),
  problem_(NULL), was_constant_set_(false)
/*****************************************************************************/
{
}

/*****************************************************************************/
void CeresSolver::Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
/*****************************************************************************/
{
  logger_ = node->get_logger();

  std::string solver_type, preconditioner_type, dogleg_type,
    trust_strategy, loss_fn, mode;
  if (!node->has_parameter("ceres_linear_solver")) {
    node->declare_parameter(
      "ceres_linear_solver",
      rclcpp::ParameterValue(std::string("SPARSE_NORMAL_CHOLESKY")));
  }
  solver_type = node->get_parameter("ceres_linear_solver").as_string();

  if (!node->has_parameter("ceres_preconditioner")) {
    node->declare_parameter(
      "ceres_preconditioner",
      rclcpp::ParameterValue(std::string("JACOBI")));
  }
  preconditioner_type = node->get_parameter("ceres_preconditioner").as_string();

  if (!node->has_parameter("ceres_dogleg_type")) {
    node->declare_parameter(
      "ceres_dogleg_type",
      rclcpp::ParameterValue(std::string("TRADITIONAL_DOGLEG")));
  }
  dogleg_type = node->get_parameter("ceres_dogleg_type").as_string();

  if (!node->has_parameter("ceres_trust_strategy")) {
    node->declare_parameter(
      "ceres_trust_strategy",
      rclcpp::ParameterValue(std::string("LM")));
  }
  trust_strategy = node->get_parameter("ceres_trust_strategy").as_string();

  if (!node->has_parameter("ceres_loss_function")) {
    node->declare_parameter(
      "ceres_loss_function",
      rclcpp::ParameterValue(std::string("None")));
  }
  loss_fn = node->get_parameter("ceres_loss_function").as_string();

  if (!node->has_parameter("mode")) {
    node->declare_parameter(
      "mode",
      rclcpp::ParameterValue(std::string("mapping")));
  }
  mode = node->get_parameter("mode").as_string();

  if (!node->has_parameter("adaptive_anchor_enabled")) {
    node->declare_parameter(
      "adaptive_anchor_enabled",
      rclcpp::ParameterValue(false));
  }
  if (!node->has_parameter("adaptive_anchor_min_weight")) {
    node->declare_parameter(
      "adaptive_anchor_min_weight",
      rclcpp::ParameterValue(1.0));
  }
  if (!node->has_parameter("adaptive_anchor_max_weight")) {
    node->declare_parameter(
      "adaptive_anchor_max_weight",
      rclcpp::ParameterValue(3.0));
  }
  if (!node->has_parameter("adaptive_anchor_decay_nodes")) {
    node->declare_parameter(
      "adaptive_anchor_decay_nodes",
      rclcpp::ParameterValue(50.0));
  }
  if (!node->has_parameter("adaptive_anchor_local_edge_max_gap")) {
    node->declare_parameter(
      "adaptive_anchor_local_edge_max_gap",
      rclcpp::ParameterValue(1));
  }
  if (!node->has_parameter("adaptive_anchor_loop_min_node_gap")) {
    node->declare_parameter(
      "adaptive_anchor_loop_min_node_gap",
      rclcpp::ParameterValue(30));
  }
  if (!node->has_parameter("adaptive_anchor_loop_evidence_min_edges")) {
    node->declare_parameter(
      "adaptive_anchor_loop_evidence_min_edges",
      rclcpp::ParameterValue(3));
  }
  if (!node->has_parameter("adaptive_anchor_loop_evidence_window_nodes")) {
    node->declare_parameter(
      "adaptive_anchor_loop_evidence_window_nodes",
      rclcpp::ParameterValue(120));
  }
  if (!node->has_parameter("adaptive_anchor_loop_evidence_min_new_node_separation")) {
    node->declare_parameter(
      "adaptive_anchor_loop_evidence_min_new_node_separation",
      rclcpp::ParameterValue(2));
  }
  if (!node->has_parameter("adaptive_anchor_loop_consistency_translation_m")) {
    node->declare_parameter(
      "adaptive_anchor_loop_consistency_translation_m",
      rclcpp::ParameterValue(0.20));
  }
  if (!node->has_parameter("adaptive_anchor_loop_consistency_yaw_deg")) {
    node->declare_parameter(
      "adaptive_anchor_loop_consistency_yaw_deg",
      rclcpp::ParameterValue(3.0));
  }
  if (!node->has_parameter("adaptive_anchor_loop_min_mahalanobis_sq")) {
    node->declare_parameter(
      "adaptive_anchor_loop_min_mahalanobis_sq",
      rclcpp::ParameterValue(9.0));
  }
  if (!node->has_parameter("adaptive_anchor_release_stage1_factor")) {
    node->declare_parameter(
      "adaptive_anchor_release_stage1_factor",
      rclcpp::ParameterValue(0.5));
  }
  if (!node->has_parameter("adaptive_anchor_release_stage2_factor")) {
    node->declare_parameter(
      "adaptive_anchor_release_stage2_factor",
      rclcpp::ParameterValue(0.0));
  }

  g_adaptive_anchor_enabled =
    node->get_parameter("adaptive_anchor_enabled").as_bool();
  g_adaptive_min_weight = std::max(
    1.0, node->get_parameter("adaptive_anchor_min_weight").as_double());
  g_adaptive_max_weight = std::max(
    g_adaptive_min_weight,
    node->get_parameter("adaptive_anchor_max_weight").as_double());
  g_adaptive_decay_nodes = std::max(
    1.0, node->get_parameter("adaptive_anchor_decay_nodes").as_double());
  g_adaptive_local_edge_max_gap = std::max(
    1, static_cast<int>(
      node->get_parameter("adaptive_anchor_local_edge_max_gap").as_int()));
  g_adaptive_loop_min_node_gap = std::max(
    g_adaptive_local_edge_max_gap + 1, static_cast<int>(
      node->get_parameter("adaptive_anchor_loop_min_node_gap").as_int()));
  g_adaptive_loop_evidence_min_edges = std::max(
    2, static_cast<int>(
      node->get_parameter("adaptive_anchor_loop_evidence_min_edges").as_int()));
  g_adaptive_loop_evidence_window_nodes = std::max(
    g_adaptive_loop_min_node_gap, static_cast<int>(
      node->get_parameter("adaptive_anchor_loop_evidence_window_nodes").as_int()));
  g_adaptive_loop_evidence_min_new_node_separation = std::max(
    1, static_cast<int>(
      node->get_parameter(
        "adaptive_anchor_loop_evidence_min_new_node_separation").as_int()));
  g_adaptive_loop_consistency_translation_m = std::max(
    0.0,
    node->get_parameter(
      "adaptive_anchor_loop_consistency_translation_m").as_double());
  g_adaptive_loop_consistency_yaw_rad =
    std::max(
      0.0,
      node->get_parameter(
        "adaptive_anchor_loop_consistency_yaw_deg").as_double()) *
    M_PI / 180.0;
  g_adaptive_loop_min_mahalanobis_sq = std::max(
    0.0,
    node->get_parameter(
      "adaptive_anchor_loop_min_mahalanobis_sq").as_double());
  g_adaptive_release_stage1_factor = std::max(
    0.0, std::min(
      1.0,
      node->get_parameter(
        "adaptive_anchor_release_stage1_factor").as_double()));
  g_adaptive_release_stage2_factor = std::max(
    0.0, std::min(
      g_adaptive_release_stage1_factor,
      node->get_parameter(
        "adaptive_anchor_release_stage2_factor").as_double()));

  g_adaptive_edges.clear();
  g_adaptive_release_regions.clear();
  g_adaptive_first_node_id = -1;
  g_adaptive_last_summary_node = -1;

  RCLCPP_INFO(
    node->get_logger(),
    "CeresSolver adaptive anchor V1: enabled=%s, weight=%.2f->%.2f, "
    "decay=%.1f, local_gap<=%d, loop_gap>=%d, evidence=%d, "
    "mahal_sq>=%.2f, release=%.2f->%.2f",
    g_adaptive_anchor_enabled ? "true" : "false",
    g_adaptive_max_weight,
    g_adaptive_min_weight,
    g_adaptive_decay_nodes,
    g_adaptive_local_edge_max_gap,
    g_adaptive_loop_min_node_gap,
    g_adaptive_loop_evidence_min_edges,
    g_adaptive_loop_min_mahalanobis_sq,
    g_adaptive_release_stage1_factor,
    g_adaptive_release_stage2_factor);

  debug_logging_ = node->get_parameter("debug_logging").as_bool();

  corrections_.clear();
  first_node_ = nodes_->end();

  // formulate problem
  angle_manifold_ = AngleManifold::Create();

  // choose loss function default squared loss (NULL)
  loss_function_ = NULL;
  if (loss_fn == "HuberLoss") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using HuberLoss loss function.");
    loss_function_ = new ceres::HuberLoss(0.7);
  } else if (loss_fn == "CauchyLoss") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using CauchyLoss loss function.");
    loss_function_ = new ceres::CauchyLoss(0.7);
  }

  // choose linear solver default CHOL
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (solver_type == "SPARSE_SCHUR") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using SPARSE_SCHUR solver.");
    options_.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (solver_type == "ITERATIVE_SCHUR") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using ITERATIVE_SCHUR solver.");
    options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else if (solver_type == "CGNR") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using CGNR solver.");
    options_.linear_solver_type = ceres::CGNR;
  }

  // choose preconditioner default Jacobi
  options_.preconditioner_type = ceres::JACOBI;
  if (preconditioner_type == "IDENTITY") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using IDENTITY preconditioner.");
    options_.preconditioner_type = ceres::IDENTITY;
  } else if (preconditioner_type == "SCHUR_JACOBI") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using SCHUR_JACOBI solver.");
    options_.preconditioner_type = ceres::SCHUR_JACOBI;
  }

  if (options_.preconditioner_type == ceres::CLUSTER_JACOBI ||
    options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
  {
    // default canonical view is O(n^2) which is unacceptable for
    // problems of this size
    options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
  }

  // choose trust region strategy default LM
  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (trust_strategy == "DOGLEG") {
    RCLCPP_INFO(
      node->get_logger(),
      "CeresSolver: Using DOGLEG trust region strategy.");
    options_.trust_region_strategy_type = ceres::DOGLEG;
  }

  // choose dogleg type default traditional
  if (options_.trust_region_strategy_type == ceres::DOGLEG) {
    options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
    if (dogleg_type == "SUBSPACE_DOGLEG") {
      RCLCPP_INFO(
        node->get_logger(),
        "CeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
      options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
    }
  }

  // a typical ros map is 5cm, this is 0.001, 50x the resolution
  options_.function_tolerance = 1e-3;
  options_.gradient_tolerance = 1e-6;
  options_.parameter_tolerance = 1e-3;

  options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps =
    options_.max_num_consecutive_invalid_steps;
  options_.num_threads = 50;
  options_.use_nonmonotonic_steps = true;
  options_.jacobi_scaling = true;

  options_.min_relative_decrease = 1e-3;

  options_.initial_trust_region_radius = 1e4;
  options_.max_trust_region_radius = 1e8;
  options_.min_trust_region_radius = 1e-16;

  options_.min_lm_diagonal = 1e-6;
  options_.max_lm_diagonal = 1e32;

  if (options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY) {
    options_.dynamic_sparsity = true;
  }

  if (mode == std::string("localization")) {
    // doubles the memory footprint, but lets us remove contraints faster
    options_problem_.enable_fast_removal = true;
  }

  // we do not want the problem definition to own these objects, otherwise they get
  // deleted along with the problem
  options_problem_.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  problem_ = new ceres::Problem(options_problem_);
}

/*****************************************************************************/
CeresSolver::~CeresSolver()
/*****************************************************************************/
{
  if (loss_function_ != NULL) {
    delete loss_function_;
  }
  if (nodes_ != NULL) {
    delete nodes_;
  }
  if (blocks_ != NULL) {
    delete blocks_;
  }
  if (problem_ != NULL) {
    delete problem_;
  }
}

/*****************************************************************************/
void CeresSolver::Compute()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (nodes_->size() == 0) {
    RCLCPP_WARN(
      logger_,
      "CeresSolver: Ceres was called when there are no nodes."
      " This shouldn't happen.");
    return;
  }

  auto rebuild_adaptive_local_blocks = [&]() {
      std::size_t rebuilt = 0;
      for (auto & item : g_adaptive_edges) {
        AdaptiveEdgeRecord & edge = item.second;
        if (!edge.temporal_local) {
          continue;
        }

        const double effective_weight = AdaptiveEffectiveWeight(edge);
        if (std::abs(effective_weight - edge.effective_weight) < 1e-9) {
          continue;
        }

        GraphIterator node1it = nodes_->find(edge.node1);
        GraphIterator node2it = nodes_->find(edge.node2);
        if (node1it == nodes_->end() || node2it == nodes_->end()) {
          continue;
        }

        problem_->RemoveResidualBlock(edge.block);

        Eigen::Matrix3d weighted_sqrt_information =
          edge.base_sqrt_information * std::sqrt(effective_weight);
        ceres::CostFunction * cost_function = PoseGraph2dErrorTerm::Create(
          edge.measurement(0),
          edge.measurement(1),
          edge.measurement(2),
          weighted_sqrt_information);
        ceres::ResidualBlockId new_block = problem_->AddResidualBlock(
          cost_function, loss_function_,
          &node1it->second(0), &node1it->second(1), &node1it->second(2),
          &node2it->second(0), &node2it->second(1), &node2it->second(2));

        const std::size_t forward_hash = GetHash(edge.node1, edge.node2);
        const std::size_t reverse_hash = GetHash(edge.node2, edge.node1);
        auto forward = blocks_->find(forward_hash);
        auto reverse = blocks_->find(reverse_hash);
        if (forward != blocks_->end()) {
          forward->second = new_block;
        } else if (reverse != blocks_->end()) {
          reverse->second = new_block;
        }

        edge.block = new_block;
        edge.effective_weight = effective_weight;
        rebuilt++;
      }
      return rebuilt;
    };

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_node_ != nodes_->end() &&
      problem_->HasParameterBlock(&first_node_->second(0)) &&
      problem_->HasParameterBlock(&first_node_->second(1)) &&
      problem_->HasParameterBlock(&first_node_->second(2))) {
    RCLCPP_DEBUG(
      logger_,
      "CeresSolver: Setting first node as a constant pose:"
      "%0.2f, %0.2f, %0.2f.", first_node_->second(0),
      first_node_->second(1), first_node_->second(2));
    problem_->SetParameterBlockConstant(&first_node_->second(0));
    problem_->SetParameterBlockConstant(&first_node_->second(1));
    problem_->SetParameterBlockConstant(&first_node_->second(2));
    was_constant_set_ = !was_constant_set_;
  }

  AdaptiveLoopEvidence pre_solve_evidence;
  if (g_adaptive_anchor_enabled) {
    pre_solve_evidence = AdaptiveDetectStrongLoopEvidence(*nodes_);
    if (pre_solve_evidence.valid &&
      AdaptiveRegionNeedsRelease(
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        g_adaptive_release_stage1_factor))
    {
      AdaptiveAddReleaseRegion(
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        g_adaptive_release_stage1_factor);
      const std::size_t rebuilt = rebuild_adaptive_local_blocks();
      RCLCPP_WARN(
        logger_,
        "Adaptive anchor strong-loop stage1: edges=%zu, region=[%d,%d], "
        "mean_mahal_sq=%.2f, release=%.2f, rebuilt=%zu",
        pre_solve_evidence.edge_count,
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        pre_solve_evidence.mean_mahalanobis_sq,
        g_adaptive_release_stage1_factor,
        rebuilt);
    }
  }

  auto solve_problem = [&]() {
      ceres::Solver::Summary local_summary;
      ceres::Solve(options_, problem_, &local_summary);
      if (debug_logging_) {
        std::cout << local_summary.FullReport() << '\n';
      }
      return local_summary;
    };

  ceres::Solver::Summary summary = solve_problem();
  if (!summary.IsSolutionUsable()) {
    RCLCPP_WARN(
      logger_, "CeresSolver: "
      "Ceres could not find a usable solution to optimize.");
    return;
  }

  // Only after the stage-1 solve do we consider the final fallback. If the
  // same region still has independent, consistent high normalized residuals,
  // remove the temporal boost there completely and solve once more.
  if (g_adaptive_anchor_enabled && pre_solve_evidence.valid) {
    const AdaptiveLoopEvidence post_solve_evidence =
      AdaptiveDetectStrongLoopEvidence(*nodes_);
    if (post_solve_evidence.valid &&
      AdaptiveRegionsOverlap(pre_solve_evidence, post_solve_evidence) &&
      AdaptiveRegionNeedsRelease(
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        g_adaptive_release_stage2_factor))
    {
      AdaptiveAddReleaseRegion(
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        g_adaptive_release_stage2_factor);
      const std::size_t rebuilt = rebuild_adaptive_local_blocks();
      RCLCPP_WARN(
        logger_,
        "Adaptive anchor strong-loop stage2 fallback: edges=%zu, "
        "region=[%d,%d], mean_mahal_sq=%.2f, release=%.2f (Toolbox 1x), "
        "rebuilt=%zu",
        post_solve_evidence.edge_count,
        pre_solve_evidence.region_start,
        pre_solve_evidence.region_end,
        post_solve_evidence.mean_mahalanobis_sq,
        g_adaptive_release_stage2_factor,
        rebuilt);

      summary = solve_problem();
      if (!summary.IsSolutionUsable()) {
        RCLCPP_WARN(
          logger_, "CeresSolver: "
          "Ceres could not find a usable solution after adaptive fallback.");
        return;
      }
    }
  }

  if (g_adaptive_anchor_enabled) {
    const int latest_node = AdaptiveLatestNodeId(*nodes_);
    if (g_adaptive_last_summary_node < 0 ||
      latest_node - g_adaptive_last_summary_node >= 50)
    {
      std::size_t local_count = 0;
      double effective_weight_sum = 0.0;
      for (const auto & item : g_adaptive_edges) {
        if (item.second.temporal_local) {
          local_count++;
          effective_weight_sum += item.second.effective_weight;
        }
      }
      const double average_weight =
        local_count > 0 ?
        effective_weight_sum / static_cast<double>(local_count) : 1.0;
      RCLCPP_INFO(
        logger_,
        "Adaptive anchor summary: latest_node=%d, local_edges=%zu, "
        "avg_effective_weight=%.3f, release_regions=%zu",
        latest_node,
        local_count,
        average_weight,
        g_adaptive_release_regions.size());
      g_adaptive_last_summary_node = latest_node;
    }
  }

  // store corrected poses
  if (!corrections_.empty()) {
    corrections_.clear();
  }
  corrections_.reserve(nodes_->size());
  karto::Pose2 pose;
  ConstGraphIterator iter = nodes_->begin();
  for (iter; iter != nodes_->end(); ++iter) {
    pose.SetX(iter->second(0));
    pose.SetY(iter->second(1));
    pose.SetHeading(iter->second(2));
    corrections_.push_back(std::make_pair(iter->first, pose));
  }
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector & CeresSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void CeresSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void CeresSolver::Reset()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  corrections_.clear();
  was_constant_set_ = false;

  g_adaptive_edges.clear();
  g_adaptive_release_regions.clear();
  g_adaptive_first_node_id = -1;
  g_adaptive_last_summary_node = -1;

  if (problem_) {
    // Note that this also frees anything the problem owns (i.e. local parameterization, cost
    // function)
    delete problem_;
  }

  if (nodes_) {
    delete nodes_;
  }

  if (blocks_) {
    delete blocks_;
  }

  nodes_ = new std::unordered_map<int, Eigen::Vector3d>();
  blocks_ = new std::unordered_map<std::size_t, ceres::ResidualBlockId>();
  problem_ = new ceres::Problem(options_problem_);
  first_node_ = nodes_->end();

  angle_manifold_ = AngleManifold::Create();
}

/*****************************************************************************/
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex)
/*****************************************************************************/
{
  // store nodes
  if (!pVertex) {
    return;
  }

  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());

  const int id = pVertex->GetObject()->GetUniqueId();

  boost::mutex::scoped_lock lock(nodes_mutex_);
  nodes_->insert(std::pair<int, Eigen::Vector3d>(id, pose2d));

  if (nodes_->size() == 1) {
    first_node_ = nodes_->find(id);
    g_adaptive_first_node_id = id;
  }
}

/*****************************************************************************/
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge)
/*****************************************************************************/
{
  // get IDs in graph for this edge
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (!pEdge) {
    return;
  }

  const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
  GraphIterator node1it = nodes_->find(node1);
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
  GraphIterator node2it = nodes_->find(node2);

  if (node1it == nodes_->end() ||
    node2it == nodes_->end() || node1it == node2it)
  {
    RCLCPP_WARN(
      logger_,
      "CeresSolver: Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo * pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  Eigen::Vector3d pose2d(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix3d information;
  information(0, 0) = precisionMatrix(0, 0);
  information(0, 1) = information(1, 0) = precisionMatrix(0, 1);
  information(0, 2) = information(2, 0) = precisionMatrix(0, 2);
  information(1, 1) = precisionMatrix(1, 1);
  information(1, 2) = information(2, 1) = precisionMatrix(1, 2);
  information(2, 2) = precisionMatrix(2, 2);
  Eigen::Matrix3d sqrt_information = information.llt().matrixU();
  const Eigen::Matrix3d base_sqrt_information = sqrt_information;

  const int node_gap = std::abs(node2 - node1);
  const bool temporal_local =
    g_adaptive_anchor_enabled &&
    node_gap <= g_adaptive_local_edge_max_gap;
  const bool loop_candidate =
    g_adaptive_anchor_enabled &&
    node_gap >= g_adaptive_loop_min_node_gap;

  double temporal_weight = 1.0;
  double effective_weight = 1.0;
  if (temporal_local) {
    const int older_node = std::min(node1, node2);
    temporal_weight = AdaptiveTemporalWeight(older_node);
    const double release_factor = AdaptiveReleaseFactorForNode(older_node);
    effective_weight =
      1.0 + release_factor * (temporal_weight - 1.0);
    sqrt_information *= std::sqrt(effective_weight);
  }

  // populate residual and parameterization for heading normalization
  ceres::CostFunction * cost_function = PoseGraph2dErrorTerm::Create(pose2d(0),
      pose2d(1), pose2d(2), sqrt_information);
  ceres::ResidualBlockId block = problem_->AddResidualBlock(
    cost_function, loss_function_,
    &node1it->second(0), &node1it->second(1), &node1it->second(2),
    &node2it->second(0), &node2it->second(1), &node2it->second(2));
  problem_->SetManifold(&node1it->second(2),
    angle_manifold_);
  problem_->SetManifold(&node2it->second(2),
    angle_manifold_);

  blocks_->insert(std::pair<std::size_t, ceres::ResidualBlockId>(
      GetHash(node1, node2), block));

  if (g_adaptive_anchor_enabled) {
    AdaptiveEdgeRecord record;
    record.node1 = node1;
    record.node2 = node2;
    record.measurement = pose2d;
    record.base_sqrt_information = base_sqrt_information;
    record.block = block;
    record.temporal_local = temporal_local;
    record.loop_candidate = loop_candidate;
    record.temporal_weight = temporal_weight;
    record.effective_weight = effective_weight;
    g_adaptive_edges[AdaptiveEdgeKey(node1, node2)] = record;

    if (temporal_local) {
      RCLCPP_DEBUG(
        logger_,
        "Adaptive anchor local edge %d-%d: temporal=%.3f effective=%.3f",
        node1, node2, temporal_weight, effective_weight);
    } else if (loop_candidate) {
      RCLCPP_DEBUG(
        logger_,
        "Adaptive anchor long-gap evidence candidate %d-%d: gap=%d weight=1.0",
        node1, node2, node_gap);
    }
  }
}

/*****************************************************************************/
void CeresSolver::RemoveNode(kt_int32s id)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator nodeit = nodes_->find(id);
  if (nodeit != nodes_->end()) {
    if (problem_->HasParameterBlock(&nodeit->second(0)) &&
        problem_->HasParameterBlock(&nodeit->second(1)) &&
        problem_->HasParameterBlock(&nodeit->second(2)))
    {
      problem_->RemoveParameterBlock(&nodeit->second(0));
      problem_->RemoveParameterBlock(&nodeit->second(1));
      problem_->RemoveParameterBlock(&nodeit->second(2));
      RCLCPP_DEBUG(
        logger_,
        "RemoveNode: Removed node id %d" ,nodeit->first);
    }
    else
    {
      RCLCPP_DEBUG(
        logger_,
        "RemoveNode: Missing parameter blocks for "
        "node id %d", nodeit->first);
    }
    AdaptiveEraseNodeMetadata(id);
    nodes_->erase(nodeit);
  } else {
    RCLCPP_ERROR(
      logger_, "RemoveNode: Failed to find node matching id %i",
      (int)id);
  }
}

/*****************************************************************************/
void CeresSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  AdaptiveEraseConstraintMetadata(sourceId, targetId);
  std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_a =
    blocks_->find(GetHash(sourceId, targetId));
  std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_b =
    blocks_->find(GetHash(targetId, sourceId));
  if (it_a != blocks_->end()) {
    problem_->RemoveResidualBlock(it_a->second);
    blocks_->erase(it_a);
  } else if (it_b != blocks_->end()) {
    problem_->RemoveResidualBlock(it_b->second);
    blocks_->erase(it_b);
  } else {
    RCLCPP_ERROR(
      logger_,
      "RemoveConstraint: Failed to find residual block for %i %i",
      (int)sourceId, (int)targetId);
  }
}

/*****************************************************************************/
void CeresSolver::ModifyNode(const int & unique_id, Eigen::Vector3d pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    double yaw_init = it->second(2);
    it->second = pose;
    it->second(2) += yaw_init;
  }
}

/*****************************************************************************/
void CeresSolver::GetNodeOrientation(const int & unique_id, double & pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    pose = it->second(2);
  }
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Vector3d> * CeresSolver::getGraph()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  return nodes_;
}

}  // namespace solver_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solver_plugins::CeresSolver, karto::ScanSolver)