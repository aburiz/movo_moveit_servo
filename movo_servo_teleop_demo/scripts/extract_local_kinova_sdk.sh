#!/usr/bin/env bash

set -euo pipefail

PKG_DIR="$(rospack find movo_servo_teleop_demo)"
REPO_ROOT="$(cd "${PKG_DIR}/.." && pwd)"
DEB_PATH="$(find "${REPO_ROOT}/kinova_api" -maxdepth 1 -type f -name 'KinovaAPI-*.deb' | sort -V | tail -n 1)"
RUNTIME_ROOT="${REPO_ROOT}/kinova_api/runtime"
LIB_DIR="${RUNTIME_ROOT}/usr/lib"

if [[ -z "${DEB_PATH}" || ! -f "${DEB_PATH}" ]]; then
  echo "Kinova SDK package not found under ${REPO_ROOT}/kinova_api" >&2
  exit 1
fi

mkdir -p "${RUNTIME_ROOT}"
rm -rf "${RUNTIME_ROOT}/usr"
dpkg-deb -x "${DEB_PATH}" "${RUNTIME_ROOT}"
mkdir -p "${LIB_DIR}"

cd "${LIB_DIR}"
ln -sfn Kinova.API.EthCommandLayerUbuntu.so EthCommandLayerUbuntu.so
ln -sfn Kinova.API.EthCommLayerUbuntu.so EthCommLayerUbuntu.so
ln -sfn Kinova.API.USBCommandLayerUbuntu.so USBCommandLayerUbuntu.so
ln -sfn Kinova.API.CommLayerUbuntu.so USBCommLayerUbuntu.so

echo "Kinova SDK extracted to: ${RUNTIME_ROOT}"
echo "Kinova SDK package: ${DEB_PATH}"
echo "Driver library path: ${LIB_DIR}"
