#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
colcon build --symlink-install --packages-skip fast_lio --cmake-args -DBUILD_TESTING=OFF "$@"
