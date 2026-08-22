#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--move] [--snapshot-source PATH] [--lama-source PATH]"
  echo
  echo "Copies the legacy LaMa folders into this repository."
  echo "Use --move to remove the old folders only after checksum verification."
}

mode=copy
snapshot_source="${HOME}/turtlebot4_lama_snapshots"
lama_source="${HOME}/lama"

while (($#)); do
  case "$1" in
    --move)
      mode=move
      shift
      ;;
    --snapshot-source)
      snapshot_source="${2:?missing value for --snapshot-source}"
      shift 2
      ;;
    --lama-source)
      lama_source="${2:?missing value for --lama-source}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(git -C "${script_dir}" rev-parse --show-toplevel)"
snapshot_destination="${project_root}/data/lama_runs"
lama_destination="${project_root}/third_party/lama"

require_safe_source() {
  local source_path="$1"
  local label="$2"

  if [[ ! -d "${source_path}" ]]; then
    echo "${label} source does not exist: ${source_path}" >&2
    exit 1
  fi

  local resolved
  resolved="$(cd -- "${source_path}" && pwd -P)"
  if [[ "${resolved}" == "/" || "${resolved}" == "${HOME}" || \
        "${resolved}" == "${project_root}" ]]; then
    echo "Refusing unsafe ${label} source: ${resolved}" >&2
    exit 1
  fi
}

sync_and_verify() {
  local source_path="$1"
  local destination_path="$2"
  local label="$3"

  require_safe_source "${source_path}" "${label}"
  mkdir -p -- "${destination_path}"
  echo "Copying ${label}: ${source_path} -> ${destination_path}"
  rsync -a -- "${source_path}/" "${destination_path}/"

  local differences
  differences="$(rsync -rcni --delete -- "${source_path}/" "${destination_path}/")"
  if [[ -n "${differences}" ]]; then
    echo "Checksum verification failed for ${label}:" >&2
    echo "${differences}" >&2
    exit 1
  fi
  echo "Verified ${label}."

  if [[ "${mode}" == "move" ]]; then
    local resolved_source
    resolved_source="$(cd -- "${source_path}" && pwd -P)"
    find "${resolved_source}" -mindepth 1 -delete
    rmdir -- "${resolved_source}"
    echo "Removed verified legacy source: ${resolved_source}"
  fi
}

command -v git >/dev/null || { echo "git is required" >&2; exit 1; }
command -v rsync >/dev/null || { echo "rsync is required" >&2; exit 1; }

sync_and_verify "${snapshot_source}" "${snapshot_destination}" "snapshot runs"
sync_and_verify "${lama_source}" "${lama_destination}" "LaMa workspace"

echo
echo "Project layout is ready:"
echo "  snapshots: ${snapshot_destination}"
echo "  LaMa:      ${lama_destination}"
echo "  mode:      ${mode}"
echo
echo "Export this in new terminals:"
echo "  export TURTLEBOT4_PROJECT_ROOT=${project_root}"
