# Frontier trap diagnostics

The hard frontier-policy changes from PR #25 were rolled back after they caused a regression: the detector rejected nearby frontiers and selected long paths. The retained changes are the latched `/robot_trapped` diagnostic, `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel` status logging, and disabled MPPI trajectory visualization for long Hospital runs.
