#!/usr/bin/env python3
"""Apply the experimental strict hard-chain Ceres modification safely.

This edits the vendored slam_toolbox/solvers/ceres_solver.cpp in-place using
validated textual anchors instead of relying on a unified-diff patch.

Hard-chain behavior when enabled by toolbox_hard_chain.launch.py:
- pose 0 is fixed forever;
- after a pose has been solved once, it is frozen permanently;
- only the newest pose remains variable at the next Ceres solve;
- with scan_buffer_size=1 and loop closure disabled, the accepted SLAM
  keyframes form a strict sequential history for this experiment.
"""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "slam_toolbox" / "solvers" / "ceres_solver.cpp"

MARKER = "CeresSolver hard chain: enabled=%s"

NAMESPACE_OLD = """namespace solver_plugins\n{\n\n/*****************************************************************************/\n"""
NAMESPACE_NEW = """namespace solver_plugins\n{\n\nnamespace\n{\n// Experimental strict sequential pose-graph mode. When enabled, every pose\n// already accepted into the graph is frozen permanently. Only the newest pose\n// remains variable during the next Ceres solve.\nbool g_hard_chain_enabled = false;\n}  // namespace\n\n/*****************************************************************************/\n"""

CONFIG_OLD = """  mode = node->get_parameter(\"mode\").as_string();\n\n  debug_logging_ = node->get_parameter(\"debug_logging\").as_bool();\n"""
CONFIG_NEW = """  mode = node->get_parameter(\"mode\").as_string();\n\n  if (!node->has_parameter(\"hard_chain_enabled\")) {\n    node->declare_parameter(\n      \"hard_chain_enabled\",\n      rclcpp::ParameterValue(false));\n  }\n  g_hard_chain_enabled =\n    node->get_parameter(\"hard_chain_enabled\").as_bool();\n\n  RCLCPP_INFO(\n    node->get_logger(),\n    \"CeresSolver hard chain: enabled=%s\",\n    g_hard_chain_enabled ? \"true\" : \"false\");\n\n  debug_logging_ = node->get_parameter(\"debug_logging\").as_bool();\n"""

COMPUTE_OLD = """  // populate contraint for static initial pose\n  if (!was_constant_set_ && first_node_ != nodes_->end() &&\n"""
COMPUTE_NEW = """  // Hard-chain experiment:\n  //   node 0 is the immutable root;\n  //   after node k has been solved once, it becomes immutable too;\n  //   only the newest node k+1 is allowed to move.\n  //\n  // With scan_buffer_size=1 and do_loop_closing=false, the active graph is a\n  // sequential chain. The newest accepted SLAM keyframe is optimized against\n  // the already-fixed history and no later optimization can move old poses.\n  if (g_hard_chain_enabled) {\n    GraphIterator newest = nodes_->begin();\n    for (GraphIterator it = nodes_->begin(); it != nodes_->end(); ++it) {\n      if (it->first > newest->first) {\n        newest = it;\n      }\n    }\n\n    for (GraphIterator it = nodes_->begin(); it != nodes_->end(); ++it) {\n      if (!problem_->HasParameterBlock(&it->second(0)) ||\n          !problem_->HasParameterBlock(&it->second(1)) ||\n          !problem_->HasParameterBlock(&it->second(2)))\n      {\n        continue;\n      }\n\n      const bool is_newest = (it == newest);\n      const bool can_move = is_newest && nodes_->size() > 1;\n\n      if (can_move) {\n        problem_->SetParameterBlockVariable(&it->second(0));\n        problem_->SetParameterBlockVariable(&it->second(1));\n        problem_->SetParameterBlockVariable(&it->second(2));\n      } else {\n        problem_->SetParameterBlockConstant(&it->second(0));\n        problem_->SetParameterBlockConstant(&it->second(1));\n        problem_->SetParameterBlockConstant(&it->second(2));\n      }\n    }\n\n    // First pose is already frozen by the hard-chain block above.\n    was_constant_set_ = true;\n\n    RCLCPP_DEBUG(\n      logger_,\n      \"CeresSolver hard chain: froze %zu historical poses; newest node=%d remains variable\",\n      nodes_->size() > 0 ? nodes_->size() - 1 : 0,\n      newest->first);\n  }\n\n  // populate contraint for static initial pose\n  if (!was_constant_set_ && first_node_ != nodes_->end() &&\n"""


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"Expected exactly one {label} anchor, found {count}. "
            "Restore ceres_solver.cpp to the repository version and retry."
        )
    return text.replace(old, new, 1)


def main() -> int:
    if not TARGET.exists():
        print(f"ERROR: target not found: {TARGET}", file=sys.stderr)
        return 1

    text = TARGET.read_text(encoding="utf-8")
    if MARKER in text:
        print("Hard-chain modification is already applied; nothing to do.")
        return 0

    try:
        text = replace_once(text, NAMESPACE_OLD, NAMESPACE_NEW, "namespace")
        text = replace_once(text, CONFIG_OLD, CONFIG_NEW, "Configure")
        text = replace_once(text, COMPUTE_OLD, COMPUTE_NEW, "Compute")
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    TARGET.write_text(text, encoding="utf-8")
    print(f"Applied hard-chain modification to: {TARGET}")
    print("Next: rebuild slam_toolbox with colcon.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
