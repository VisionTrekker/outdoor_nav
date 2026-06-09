#!/bin/bash
# Format C/C++ files in the current git working tree (uncommitted changes only).
# Skips: build/, cmake-build-debug/, Thirdparty/, 3rd/, .clang-format itself, .git/.
# Uses the project's .clang-format (clang-format auto-detects the nearest one).
set -euo pipefail

script_dir=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
cd "$script_dir"

# -m = modified (staged or unstaged), --others = untracked, pathspec excludes
# build/cmake-build-debug/Thirdparty/3rd dirs and the clang-format config + git dir.
modified_files=$(git status --porcelain -- \
    ':!build' ':!build/' \
    ':!cmake-build-debug' ':!cmake-build-debug/' \
    ':!Thirdparty' ':!Thirdparty/' \
    ':!3rd' ':!3rd/' \
    ':!.clang-format' \
    ':!.git' ':!.git/' \
    | awk '{print $2}')

while IFS= read -r src_file; do
    [[ -z "$src_file" ]] && continue
    [[ -f "$src_file" ]] || continue
    case "${src_file##*.}" in
        h|c|hpp|cc|cpp|tpp)
            echo "clang-format $src_file"
            clang-format -i "$src_file"
            ;;
    esac
done <<< "$modified_files"
