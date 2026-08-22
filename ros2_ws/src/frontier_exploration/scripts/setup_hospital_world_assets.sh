#!/usr/bin/env bash
set -euo pipefail

SOURCE_REPO="https://github.com/hoale-motion/Robot_Omni_Navigation.git"
CACHE_ROOT="${HOME}/.cache/turtlebot4_project/hospital_world"
MODELS_DIR="${CACHE_ROOT}/models"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

mkdir -p "${CACHE_ROOT}"

echo "Downloading Hospital World structural assets..."
git clone --depth 1 --filter=blob:none --sparse "${SOURCE_REPO}" "${TMP_DIR}/repo"
git -C "${TMP_DIR}/repo" sparse-checkout set \
  src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_floor \
  src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_walls

rm -rf "${MODELS_DIR}"
mkdir -p "${MODELS_DIR}"
cp -a "${TMP_DIR}/repo/src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_floor" "${MODELS_DIR}/"
cp -a "${TMP_DIR}/repo/src/hospital_robot/worlds/models/aws_robomaker_hospital_floor_01_walls" "${MODELS_DIR}/"

echo "Hospital assets installed at: ${MODELS_DIR}"
echo "Use this path as GZ_SIM_RESOURCE_PATH when launching the hospital world."
