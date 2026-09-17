#!/usr/bin/env python3
"""MapEx + frozen Way2 early stopping for prospective validation.

This variant deliberately keeps ``mapex.py`` unchanged as the MapEx baseline.
It inherits all MapEx prediction/frontier/visibility/scoring helpers and the
shared ``nf_basic.py`` navigation, recovery, revalidation, and completion
machinery. The only policy addition is the frozen Way2 stopping decision placed
after MapEx execution/selectability filters and before choosing/sending the next
Nav2 frontier goal.

Frozen Way2 rule:
- F_t = selectable MapEx frontiers after distance preference / near fallback
  and planner-blocking suppression.
- R_t = max_f information_gain(f) / distance_m(f), over f in F_t.
- U_t = max_f visible_unknown_cells(f) / distance_m(f), over f in F_t.
- base_valid = (R_t <= 0.30) and (U_t <= 10.0 cells/m).
- Count only consecutive *evaluable* MapEx decisions.
- 1st consecutive base-valid decision: continue.
- 2nd consecutive base-valid decision:
    * candidate_count <= 1 -> early stop.
    * candidate_count > 1  -> continue and require one more valid decision.
- 3rd consecutive base-valid decision -> early stop.
- Any base-invalid evaluable decision resets the Way2 valid count to zero.

The frozen thresholds must not be retuned on the existing development runs.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from mapex import MapExExplorer, ros_occupancy_to_mapex
from nf_basic import MAP_FRAME, MIN_DISTANCE_THRESHOLD


WAY2_R_THRESHOLD = 0.30
WAY2_U_THRESHOLD_CELLS_PER_M = 10.0
WAY2_SINGLE_CANDIDATE_CUTOFF = 1
WAY2_SECOND_CONFIRMATION = 2
WAY2_THIRD_CONFIRMATION = 3


class MapExWay2Explorer(MapExExplorer):
    """MapEx baseline policy plus the frozen Way2 online early-stop gate."""

    def __init__(self) -> None:
        super().__init__()
        self.way2_valid_count = 0

        self.get_logger().info(
            "MapEx Way2 enabled "
            f"(R_t<={WAY2_R_THRESHOLD:.2f}, "
            f"U_t<={WAY2_U_THRESHOLD_CELLS_PER_M:.1f} cells/m, "
            f"single-candidate cutoff<={WAY2_SINGLE_CANDIDATE_CUTOFF}, "
            "confirmation=2 decisions if <=1 candidate at the second "
            "confirmation, otherwise 3; frozen prospective-validation rule)"
        )

    @staticmethod
    def _way2_u_value(evaluation: dict) -> float:
        """visible_unknown_cells(f) / distance_m(f), matching the frozen audit."""
        distance_m = max(float(evaluation["distance_m"]), 1e-6)
        return float(evaluation["visible_unknown_cells"]) / distance_m

    def evaluate_way2_stop(
        self,
        selectable_evaluations: list[dict],
    ) -> tuple[bool, dict]:
        """Evaluate the frozen Way2 state machine on selectable F_t only."""
        if not selectable_evaluations:
            raise ValueError(
                "Way2 must only be evaluated when selectable frontiers exist"
            )

        candidate_count = len(selectable_evaluations)
        r_t = max(float(candidate["score"]) for candidate in selectable_evaluations)
        u_t = max(
            self._way2_u_value(candidate)
            for candidate in selectable_evaluations
        )
        base_valid = (
            r_t <= WAY2_R_THRESHOLD
            and u_t <= WAY2_U_THRESHOLD_CELLS_PER_M
        )

        if not base_valid:
            self.way2_valid_count = 0
            should_stop = False
            action = "continue_base_invalid_reset_confirmation"
        else:
            self.way2_valid_count += 1

            if self.way2_valid_count == 1:
                should_stop = False
                action = "continue_first_valid_confirmation"
            elif self.way2_valid_count == WAY2_SECOND_CONFIRMATION:
                if candidate_count <= WAY2_SINGLE_CANDIDATE_CUTOFF:
                    should_stop = True
                    action = "stop_second_valid_single_candidate"
                else:
                    should_stop = False
                    action = "continue_require_third_valid_confirmation"
            elif self.way2_valid_count >= WAY2_THIRD_CONFIRMATION:
                # A third consecutive base-valid evaluable decision is terminal,
                # regardless of the candidate count at that third decision.
                should_stop = True
                action = "stop_third_consecutive_valid_confirmation"
            else:
                raise RuntimeError(
                    f"Unexpected Way2 valid count: {self.way2_valid_count}"
                )

        metrics = {
            "r_t": float(r_t),
            "u_t_cells_per_m": float(u_t),
            "candidate_count": int(candidate_count),
            "base_valid": bool(base_valid),
            "valid_count": int(self.way2_valid_count),
            "action": action,
        }
        return should_stop, metrics

    def publish_way2_check(self, metrics: dict) -> None:
        """Publish/log every evaluable Way2 decision for later audit."""
        self.publish_status(
            "MAPEX_WAY2_CHECK",
            reason=metrics["action"],
            decision_id=self.mapex_decision_id,
            way2_r_t=metrics["r_t"],
            way2_r_threshold=WAY2_R_THRESHOLD,
            way2_u_t_cells_per_m=metrics["u_t_cells_per_m"],
            way2_u_threshold_cells_per_m=WAY2_U_THRESHOLD_CELLS_PER_M,
            candidate_count=metrics["candidate_count"],
            way2_base_valid=metrics["base_valid"],
            way2_valid_count=metrics["valid_count"],
        )
        self.get_logger().info(
            f"MapEx Way2 decision {self.mapex_decision_id}: "
            f"R_t={metrics['r_t']:.6f} "
            f"(<= {WAY2_R_THRESHOLD:.2f}), "
            f"U_t={metrics['u_t_cells_per_m']:.3f} cells/m "
            f"(<= {WAY2_U_THRESHOLD_CELLS_PER_M:.1f}), "
            f"candidates={metrics['candidate_count']}, "
            f"base_valid={metrics['base_valid']}, "
            f"valid_count={metrics['valid_count']}, "
            f"action={metrics['action']}"
        )

    def mark_way2_early_stop(self, metrics: dict) -> None:
        """Publish terminal completion without using baseline completion evidence."""
        if self.completed:
            return

        # This method is reached only at an evaluable decision boundary:
        # there is no active navigation goal and no new goal has been issued.
        self.completion_reason = "way2_early_stop"
        self.completed = True
        self.revalidation_active = False
        self.revalidation_signature = None
        self.main_goal = None
        self.latest_main_plan = None
        self.clear_goal_markers()

        empty_path = Path()
        empty_path.header.frame_id = MAP_FRAME
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

        complete = Bool()
        complete.data = True
        self.completion_pub.publish(complete)

        self.publish_status(
            "COMPLETE",
            reason="way2_early_stop",
            decision_id=self.mapex_decision_id,
            way2_r_t=metrics["r_t"],
            way2_r_threshold=WAY2_R_THRESHOLD,
            way2_u_t_cells_per_m=metrics["u_t_cells_per_m"],
            way2_u_threshold_cells_per_m=WAY2_U_THRESHOLD_CELLS_PER_M,
            candidate_count=metrics["candidate_count"],
            way2_base_valid=metrics["base_valid"],
            way2_valid_count=metrics["valid_count"],
            way2_stop_rule=metrics["action"],
        )

        self.get_logger().warn(
            "================ WAY2 EARLY STOP =====================\n"
            "Reason: frozen Way2 early-stopping rule satisfied.\n"
            f"Decision: {self.mapex_decision_id}; "
            f"R_t={metrics['r_t']:.6f}; "
            f"U_t={metrics['u_t_cells_per_m']:.3f} cells/m; "
            f"selectable candidates={metrics['candidate_count']}; "
            f"valid_count={metrics['valid_count']}.\n"
            "No new Nav2 exploration goal will be issued.\n"
            "======================================================"
        )

    def exploration_step(self):
        """Run one MapEx decision with Way2 inserted before next-goal selection."""
        if self.completed:
            return
        if self.map_msg is None or self.goal_active or self.revalidation_active:
            return

        # Recovery retries are not new evaluable Way2 decisions.
        if self.main_goal is not None:
            self.reset_completion_verification("main_frontier_still_pending")
            self.revalidation_signature = None
            x, y = self.main_goal
            self.get_logger().info(
                f"Retrying MapEx main frontier after recovery: x={x:.2f}, y={y:.2f}"
            )
            self.send_navigation_goal(x, y, mode="main")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        resolution = float(self.map_msg.info.resolution)
        if not math.isclose(
            resolution,
            self.mapex_resolution_m,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_resolution_mismatch")
            self.publish_status(
                "MAPEX_ERROR",
                reason="policy_grid_resolution_mismatch",
                map_resolution_m=resolution,
                required_resolution_m=self.mapex_resolution_m,
            )
            self.get_logger().error(
                f"MapEx config requires /map at {self.mapex_resolution_m:.2f} m/cell; "
                f"got {resolution:.6f} m/cell. Not issuing a goal."
            )
            return

        grid = self.get_current_map()
        observed_map = ros_occupancy_to_mapex(grid)
        self.mapex_decision_id += 1
        decision_start = time.perf_counter()

        try:
            prediction_start = time.perf_counter()
            (
                predictions,
                padded_observed,
                pad_top,
                pad_left,
            ) = self.predict_maps(observed_map)
            mean_map = self.compute_mean_map(predictions, padded_observed)
            variance_map = self.compute_variance_map(
                predictions,
                padded_observed,
            )
            prediction_s = time.perf_counter() - prediction_start

            if (
                mean_map.shape != padded_observed.shape
                or variance_map.shape != padded_observed.shape
            ):
                raise RuntimeError(
                    f"LaMa shape mismatch: obs={padded_observed.shape}, "
                    f"mean={mean_map.shape}, variance={variance_map.shape}"
                )

            self.last_mean_map = mean_map
            self.last_variance_map = variance_map

            # Flowchart: extraction -> strict size filter -> candidate evaluation.
            frontier_cells = self.detect_frontier_cells(grid)
            regions = self.connected_components(frontier_cells)
            regions = self.filter_small_clusters(regions)

            if not regions:
                # No selectable F_t can be formed, so this is not a Way2
                # evaluable decision. Preserve baseline terminal handling.
                self.last_candidate_metrics = []
                self.clear_goal_markers()
                self.revalidation_signature = None
                self.observe_no_frontier_terminal_state()
                return

            frontiers = self.compute_frontier_centroids(regions)
            evaluations, _scoring_s = self.evaluate_frontiers(
                frontiers=frontiers,
                mean_map=mean_map,
                variance_map=variance_map,
                padded_observed=padded_observed,
                robot_pose=robot_pose,
                pad_top=pad_top,
                pad_left=pad_left,
                prediction_s=prediction_s,
            )
        except Exception as exc:  # noqa: BLE001
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_scoring_failed")
            self.publish_status(
                "MAPEX_ERROR",
                reason="prediction_visibility_or_scoring_failed",
                decision_id=self.mapex_decision_id,
                error=str(exc),
            )
            self.get_logger().error(
                f"MapEx decision {self.mapex_decision_id} failed: {exc}"
            )
            return

        self.last_candidate_metrics = evaluations
        if not evaluations:
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_no_scoreable_candidate")
            self.publish_status(
                "MAPEX_ERROR",
                reason="frontier_regions_exist_but_no_scoreable_candidate",
                decision_id=self.mapex_decision_id,
                frontier_regions=len(regions),
            )
            return

        # Flowchart: apply MapEx execution/selectability filters.
        distant_evaluations = [
            evaluation
            for evaluation in evaluations
            if evaluation["distance_m"] >= MIN_DISTANCE_THRESHOLD
        ]
        near_frontier_fallback = not distant_evaluations
        distance_eligible_evaluations = (
            distant_evaluations if distant_evaluations else evaluations
        )

        if near_frontier_fallback:
            self.get_logger().info(
                "All current MapEx frontier representatives are closer than "
                f"{MIN_DISTANCE_THRESHOLD:.2f} m; enabling near-frontier fallback."
            )

        selectable_evaluations = []
        suppressed_planner_blocked = 0
        for evaluation in distance_eligible_evaluations:
            if self.is_planner_blocked_suppressed(
                evaluation["x"],
                evaluation["y"],
            ):
                suppressed_planner_blocked += 1
            else:
                selectable_evaluations.append(evaluation)

        all_execution_candidates = [
            self.to_execution_candidate(evaluation)
            for evaluation in distance_eligible_evaluations
        ]

        if not selectable_evaluations:
            # Again, no Way2 evaluation here: preserve baseline revalidation /
            # terminal handling and leave Way2 confirmation state unchanged.
            self.clear_goal_markers()
            if suppressed_planner_blocked == len(distance_eligible_evaluations):
                self.maybe_start_planner_revalidation(all_execution_candidates)
                return
            self.revalidation_signature = None
            self.reset_completion_verification("mapex_frontier_region_present")
            return

        # F_t is now exactly the selectable frontier set from the frozen rule.
        self.revalidation_signature = None
        self.reset_completion_verification("planner_candidate_available")

        should_stop, way2_metrics = self.evaluate_way2_stop(
            selectable_evaluations
        )
        self.publish_way2_check(way2_metrics)

        if should_stop:
            self.mark_way2_early_stop(way2_metrics)
            return

        # Way2 says CONTINUE: from here onward use the unchanged MapEx ranking
        # and shared Nav2 execution path.
        selected_metric = self.select_best_frontier(selectable_evaluations)
        if selected_metric is None:
            return

        selectable_execution_candidates = [
            self.to_execution_candidate(evaluation)
            for evaluation in selectable_evaluations
        ]
        selected_execution_candidate = self.to_execution_candidate(selected_metric)
        self.publish_goal_markers(
            selectable_execution_candidates,
            selected_execution_candidate,
        )

        x = float(selected_metric["x"])
        y = float(selected_metric["y"])
        region_size = int(selected_metric["region_size"])
        decision_s = time.perf_counter() - decision_start

        self.get_logger().info(
            f"MapEx decision {self.mapex_decision_id}: selected x={x:.2f}, y={y:.2f}, "
            f"IG={selected_metric['information_gain']:.6f}, "
            f"distance={selected_metric['distance_m']:.2f} m, "
            f"score={selected_metric['score']:.6f}, region={region_size}, "
            f"candidates={len(selectable_evaluations)}, "
            f"near_fallback={near_frontier_fallback}, compute={decision_s:.2f} s"
        )
        self.publish_status(
            "MAPEX_SELECTED",
            reason="highest_information_gain_over_euclidean_distance",
            decision_id=self.mapex_decision_id,
            candidate_count=len(selectable_evaluations),
            suppressed_planner_blocked=suppressed_planner_blocked,
            preferred_min_distance_m=MIN_DISTANCE_THRESHOLD,
            near_frontier_fallback=near_frontier_fallback,
            selected_row=int(selected_metric["row"]),
            selected_col=int(selected_metric["col"]),
            target_x=round(x, 4),
            target_y=round(y, 4),
            information_gain=selected_metric["information_gain"],
            distance_m=selected_metric["distance_m"],
            score=selected_metric["score"],
            computation_s=round(decision_s, 4),
            way2_r_t=way2_metrics["r_t"],
            way2_u_t_cells_per_m=way2_metrics["u_t_cells_per_m"],
            way2_base_valid=way2_metrics["base_valid"],
            way2_valid_count=way2_metrics["valid_count"],
        )

        self.main_goal = (x, y)
        self.latest_main_plan = None
        self.recovery_count = 0
        self.send_navigation_goal(x, y, mode="main")


def _mapex_way2_init_args(args):
    """Force a dedicated ROS node name while preserving MapEx parameters."""
    effective_args = list(sys.argv if args is None else args)
    effective_args.extend(["--ros-args", "-r", "__node:=mapex_way2_explorer"])
    return effective_args


def main(args=None):
    rclpy.init(args=_mapex_way2_init_args(args))
    node = None
    try:
        node = MapExWay2Explorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
