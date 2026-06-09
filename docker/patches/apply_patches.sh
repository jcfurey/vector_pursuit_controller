#!/usr/bin/env bash
# Workarounds for building navigation2 'main' against the ROS 2 Lyrical release.
#
# Both items below are skews between bleeding-edge nav2 main and the toolchain
# Lyrical actually shipped. They should become unnecessary once Nav2 cuts a
# Lyrical release (its 'main' currently tracks Rolling, which is ahead of
# Lyrical's BehaviorTree.CPP and pairs with GCC 15).
#
# NOTE: run this AFTER `rosdep install` — patch (1) edits the BehaviorTree.CPP
# headers that rosdep installs, so it is a no-op if run beforehand.
#
# Usage: apply_patches.sh [WORKSPACE_SRC_DIR]   (default: /opt/ws/src)
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
WS_SRC="${1:-/opt/ws/src}"
BT_HEADER="/opt/ros/${ROS_DISTRO}/include/behaviortree_cpp/bt_factory.h"
NAV2_PKG_CMAKE="${WS_SRC}/navigation2/nav2_common/cmake/nav2_package.cmake"

# 1) BehaviorTree.CPP 4.9.0 (Lyrical) lacks the public Tree::wakeUpSignal()
#    getter that nav2 main now calls. Add the inline accessor exactly as
#    BT.CPP master defines it (it returns the already-present private member).
if [[ ! -f "$BT_HEADER" ]]; then
  echo "[patch] WARNING: $BT_HEADER not found — is BehaviorTree.CPP installed yet?" >&2
  echo "[patch]          run this after 'rosdep install'." >&2
elif grep -q "wakeUpSignal() const" "$BT_HEADER"; then
  echo "[patch] BT.CPP Tree::wakeUpSignal() already present"
else
  sed -i 's|^  void emitWakeUpSignal();|  void emitWakeUpSignal();\n\n  [[nodiscard]] std::shared_ptr<WakeUpSignal> wakeUpSignal() const { return wake_up_; }|' "$BT_HEADER"
  echo "[patch] added Tree::wakeUpSignal() getter to $BT_HEADER"
fi

# 2) GCC 15 (Ubuntu 26.04 / Resolute) emits a spurious -Werror=null-dereference
#    inside generated rosidl message comparison operators. nav2_package.cmake
#    builds with -Werror; append -Wno-error so warnings stay visible but the
#    build is not aborted by this false positive.
if [[ -f "$NAV2_PKG_CMAKE" ]] && ! grep -q -- "-Wno-error" "$NAV2_PKG_CMAKE"; then
  sed -i 's|\(-Wpedantic -Werror\)|\1 -Wno-error|' "$NAV2_PKG_CMAKE"
  echo "[patch] appended -Wno-error in $NAV2_PKG_CMAKE"
else
  echo "[patch] -Wno-error already present (or nav2_package.cmake not found)"
fi
