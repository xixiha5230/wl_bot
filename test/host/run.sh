#!/bin/sh
# Build and run the host-side firmware unit tests (no ESP-IDF required).
set -eu

root="$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)"
cc="${CC:-cc}"
out="${TMPDIR:-/tmp}/robot_math_test"

"$cc" -std=c11 -Wall -Wextra -O2 -I"$root/main" -o "$out" "$root/test/host/test_robot_math.c" -lm
"$out"
