#!/usr/bin/env bash
set -euo pipefail

SOURCE_REPO="https://github.com/hoale-motion/Robot_Omni_Navigation.git"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAPEX_LAB_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
MODELS_DIR="${MAPEX_LAB_ROOT}/map/models"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

echo "Downloading Hospital World structural assets..."
git clone --depth 1 --filter=blob:none --sparse "${SOURCE_REPO}" "${TMP_DIR}/repo"
git -C "${TMP_DIR}/repo" sparse-checkout set \
  src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_floor \
  src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_walls \
  src/hospital_robot/worlds/models/aws_robomaker_hospital_curtain_closed_01

rm -rf "${MODELS_DIR}"
mkdir -p "${MODELS_DIR}"
cp -a "${TMP_DIR}/repo/src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_floor" "${MODELS_DIR}/"
cp -a "${TMP_DIR}/repo/src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_walls" "${MODELS_DIR}/"
cp -a "${TMP_DIR}/repo/src/hospital_robot/worlds/models/aws_robomaker_hospital_curtain_closed_01" "${MODELS_DIR}/"

echo "Hospital assets installed at: ${MODELS_DIR}"
echo "Normal Hospital launch files use this directory automatically."
