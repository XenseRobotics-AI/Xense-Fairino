#!/usr/bin/env bash
set -euo pipefail
CONTROLLER_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CONTROLLER_PACKAGE="$CONTROLLER_ROOT/ros2/fairino_lookahead_controller"
PLANNER_PACKAGE="$CONTROLLER_ROOT/ros2/fairino_straight_planner"
CONTROLLER_CXX="${CXX:-c++}"
if ! command -v "$CONTROLLER_CXX" >/dev/null 2>&1; then
  echo '需要 C++17 编译器（clang++ 或 g++）；可通过 CXX 指定。' >&2
  exit 2
fi
CONTROLLER_TEMP=$(mktemp -d)
trap 'rm -rf -- "$CONTROLLER_TEMP"' EXIT
for controller_test in core collision coordination; do
  "$CONTROLLER_CXX" -std=c++17 -Wall -Wextra -Werror -pedantic \
    -I "$CONTROLLER_PACKAGE/include" \
    "$CONTROLLER_PACKAGE/test/test_${controller_test}.cpp" \
    -o "$CONTROLLER_TEMP/test_${controller_test}"
  "$CONTROLLER_TEMP/test_${controller_test}"
done
"$CONTROLLER_CXX" -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$PLANNER_PACKAGE/include" \
  "$PLANNER_PACKAGE/test/test_core.cpp" \
  -o "$CONTROLLER_TEMP/test_straight_planner"
"$CONTROLLER_TEMP/test_straight_planner"

"$CONTROLLER_CXX" -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$PLANNER_PACKAGE/include" \
  "$PLANNER_PACKAGE/test/test_planner_api.cpp" \
  -o "$CONTROLLER_TEMP/test_planner_api"
"$CONTROLLER_TEMP/test_planner_api"
