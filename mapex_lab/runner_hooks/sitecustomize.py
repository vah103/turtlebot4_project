"""Runner-only ROS spin hook for automatic experiment completion.

Loaded through PYTHONPATH only by the repository-level ``./run`` wrapper.
It leaves the exploration policy untouched: the node itself still decides when
``completed`` becomes true.  The hook only turns that terminal state into a
normal process exit so recorder finalization, offline evaluation, and launcher
cleanup can proceed automatically.
"""

from __future__ import annotations

import os


if os.environ.get("MAPEX_RUNNER_AUTO_EXIT") == "1":
    try:
        import rclpy
    except Exception:
        # The LaMa Conda worker inherits the runner environment but does not need
        # ROS.  Never make that interpreter depend on rclpy just because this
        # hook is present on PYTHONPATH.
        rclpy = None

    if rclpy is not None:
        _original_spin_once = rclpy.spin_once

        def _spin_until_exploration_complete(node, executor=None):
            """Spin normally until ROS shuts down or the explorer is complete."""
            while rclpy.ok():
                _original_spin_once(node, executor=executor, timeout_sec=0.2)

                if not bool(getattr(node, "completed", False)):
                    continue

                # Recorder entrypoints normally finalize only after Ctrl+C.
                # Finalize here while ROS is still alive so a natural completion
                # is recorded accurately and all files are flushed before the
                # ordinary entrypoint cleanup/offline evaluation runs.
                finalize = getattr(node, "finalize", None)
                finalized = bool(getattr(node, "finalized", False))
                if callable(finalize) and not finalized:
                    finalize("exploration_complete")
                return

        rclpy.spin = _spin_until_exploration_complete
