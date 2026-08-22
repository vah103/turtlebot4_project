# Third-party workspaces

`third_party/lama_upstream/` is a Git submodule pinned to the official
`advimman/lama` repository. It lets reviewers and other ChatGPT sessions inspect
the exact upstream source revision without vendoring the source into this repo.

`third_party/lama/` remains the machine-local runtime workspace containing the
downloaded model, environment and generated outputs. It is ignored by Git so
the existing Dell installation is not overwritten by the submodule.

Use `scripts/lama/migrate_local_assets.sh` to copy or move the existing
`~/lama` workspace here.
