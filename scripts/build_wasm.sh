#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EMS_DIR="${ROOT}/.emsdk"
if [[ ! -d "${EMS_DIR}" ]]; then
  git clone https://github.com/emscripten-core/emsdk.git "${EMS_DIR}"
  git -C "${EMS_DIR}" checkout 1693ce7359fc9694f198694f902a2cc061239b86
fi
# shellcheck source=/dev/null
source "${EMS_DIR}/emsdk_env.sh"
"${EMS_DIR}/emsdk" install 7.0.0
"${EMS_DIR}/emsdk" activate 7.0.0
source "${EMS_DIR}/emsdk_env.sh"
emcc --version
cmake -S "${ROOT}" -B "${ROOT}/build-wasm" -DSGS_BUILD_TESTS=OFF -DSGS_BUILD_WASM=ON \
  -DCMAKE_TOOLCHAIN_FILE="${EMS_DIR}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake"
cmake --build "${ROOT}/build-wasm" -j"$(nproc)"
mkdir -p "${ROOT}/web/public"
cp "${ROOT}/build-wasm/cpp/sgs.js" "${ROOT}/web/public/"
cp "${ROOT}/build-wasm/cpp/sgs.wasm" "${ROOT}/web/public/"
