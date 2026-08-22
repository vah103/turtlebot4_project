# Third-party workspaces

`third_party/lama/` is the project-local LaMa checkout/workspace. Its full
contents, downloaded model checkpoints, environments and generated outputs are
ignored by Git. This avoids vendoring a large upstream repository or model
payload while keeping every runtime path inside `turtlebot4_project`.

Use `scripts/lama/migrate_local_assets.sh` to copy or move the existing
`~/lama` workspace here.
