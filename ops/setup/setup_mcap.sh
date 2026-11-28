#!/usr/bin/env bash
# Install only into a dedicated collection dependency directory.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
target="${COLLECTION_MCAP_DEPS:-$repo/.runtime/mcap-python}"
python="${COLLECTION_MCAP_PYTHON:-python3}"
"$python" -m pip install --target "$target" -r "$repo/data_collection/config/mcap-requirements.txt"
g++ -O3 -shared -fPIC "$repo/ros_ws/src/teleop_data_collector/teleop_data_collector/rvl_encode.cpp" -o "$target/mcap-rvl.so"
echo "MCAP dependencies: $target"
