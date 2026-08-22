# Frontier trap fix

This change keeps frontier goals farther inside known free space, requires local maneuvering clearance, preserves the approach heading instead of forcing the robot to face the frontier, stops repeated same-pose navigation failures with a latched `/robot_trapped` diagnostic, records the `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel` chain in the status JSONL, and disables MPPI trajectory visualization during long Hospital runs.
