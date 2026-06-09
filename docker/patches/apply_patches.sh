#!/usr/bin/env bash
# Workarounds for building navigation2 'main' against the ROS 2 Lyrical release.
#
# Both items below are skews between bleeding-edge nav2 main and the toolchain
# Lyrical actually shipped. They should become unnecessary once Nav2 cuts a
# Lyrical release (its 'main' currently tracks Rolling, which is ahead of
# Lyrical's BehaviorTree.CPP and pairs with GCC 15).
#
# NOTE: run this AFTER `rosdep install` — patch (1) edits the BehaviorTree.CPP
# headers that rosdep installs. (If you `apt upgrade` inside a container later,
# patch (1) is reverted by the new package; re-run this script before
# rebuilding.)
#
# Each patch verifies its edit landed and fails loudly otherwise, so upstream
# drift breaks here with a clear message instead of minutes later in colcon.
#
# Usage: apply_patches.sh [WORKSPACE_SRC_DIR]   (default: /opt/ws/src)
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
WS_SRC="${1:-/opt/ws/src}"
BT_HEADER="/opt/ros/${ROS_DISTRO}/include/behaviortree_cpp/bt_factory.h"
NAV2_PKG_CMAKE="${WS_SRC}/navigation2/nav2_common/cmake/nav2_package.cmake"

fail() { echo "[patch] ERROR: $*" >&2; exit 1; }

# 1) BehaviorTree.CPP 4.9.0 (Lyrical) lacks the public Tree::wakeUpSignal()
#    getter that nav2 main now calls. Add the inline accessor exactly as
#    BT.CPP master defines it (it returns the already-present private member).
[[ -f "$BT_HEADER" ]] || fail "$BT_HEADER not found — run this after 'rosdep install'"
if grep -q "wakeUpSignal() const" "$BT_HEADER"; then
  echo "[patch] BT.CPP Tree::wakeUpSignal() already present"
else
  sed -i 's|^  void emitWakeUpSignal();|  void emitWakeUpSignal();\n\n  [[nodiscard]] std::shared_ptr<WakeUpSignal> wakeUpSignal() const { return wake_up_; }|' "$BT_HEADER"
  grep -q "wakeUpSignal() const" "$BT_HEADER" \
    || fail "sed anchor 'void emitWakeUpSignal();' not found in $BT_HEADER — upstream BT.CPP changed; update this patch"
  echo "[patch] added Tree::wakeUpSignal() getter to $BT_HEADER"
fi

# 2) GCC 15 (Ubuntu 26.04 / Resolute) turns several warnings in nav2 and
#    rosidl-generated code into errors under nav2's -Werror. Demote exactly
#    the categories observed in a full build of the controller's dependency
#    closure — everything else stays -Werror:
#      deprecated-declarations  (std::atomic_load on shared_ptr, etc.)
#      free-nonheap-object      (GCC 15 false positives)
#      null-dereference         (false positive in generated msg operator==)
#      maybe-uninitialized
GCC15_NO_ERROR="-Wno-error=deprecated-declarations -Wno-error=free-nonheap-object -Wno-error=null-dereference -Wno-error=maybe-uninitialized"
[[ -f "$NAV2_PKG_CMAKE" ]] || fail "$NAV2_PKG_CMAKE not found — is navigation2 cloned under $WS_SRC?"
if grep -q -- "-Wno-error=" "$NAV2_PKG_CMAKE"; then
  echo "[patch] scoped -Wno-error= flags already present in nav2_package.cmake"
else
  sed -i "s|\(-Wpedantic -Werror\)|\1 ${GCC15_NO_ERROR}|" "$NAV2_PKG_CMAKE"
  grep -q -- "-Wno-error=" "$NAV2_PKG_CMAKE" \
    || fail "sed anchor '-Wpedantic -Werror' not found in $NAV2_PKG_CMAKE — upstream nav2 changed its flags; update this patch"
  echo "[patch] appended scoped -Wno-error= flags in $NAV2_PKG_CMAKE"
fi
