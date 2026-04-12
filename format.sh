#!/usr/bin/env bash

set -euo pipefail

# clang-format v15 is required.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_root"

find . \
  \( -path './.git' -o -path './build' -o -path './third_party' -o -path './thirdparty' \) -prune -o \
  -type f \
  \( -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' -o -name '*.hxx' \) \
  ! -name '*.pb.cc' \
  ! -name '*.pb.h' \
  -print0 | xargs -0 -r clang-format -i --style=file --fallback-style=none
