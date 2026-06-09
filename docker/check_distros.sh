#!/usr/bin/env bash
# Verify vector_pursuit_controller usability across the supported ROS 2
# distros (see "ROS 2 distro support" in UPSTREAM.md):
#
#   humble  — released binaries from the upstream `master` branch (1.0.2)
#   jazzy   — released binaries from the `jazzy` branch (2.0.0)
#   kilted  — no release; the `jazzy` branch source-builds against kilted's
#             nav2 binaries
#   lyrical — no nav2 binaries at all; THIS branch builds via ./docker/build.sh
#             (run it separately — it is the heavyweight from-source path)
#
# Each check runs in the official ros:<distro>-ros-base image and reports
# PASS/FAIL. Requires docker and (for the kilted check) this git repo.
set -uo pipefail
cd "$(dirname "$0")/.."          # repository root

failures=0
report() {  # report <distro> <status-text>
  printf '  %-8s %s\n' "$1" "$2"
}

binary_check() {  # binary_check <distro>
  local d="$1"
  if docker run --rm "ros:${d}-ros-base" bash -c "
      apt-get update -qq >/dev/null 2>&1 &&
      apt-get install -y -qq ros-${d}-vector-pursuit-controller >/dev/null 2>&1 &&
      test -f /opt/ros/${d}/lib/libvector_pursuit_controller.so" >/dev/null 2>&1
  then
    report "$d" "PASS (binary install)"
  else
    report "$d" "FAIL (binary install)"
    failures=$((failures + 1))
  fi
}

echo "[check] humble and jazzy: released binaries"
binary_check humble &
binary_check jazzy &
wait

echo "[check] kilted: source build of the 'jazzy' branch"
worktree="$(mktemp -d)"
trap 'git worktree remove --force "$worktree" >/dev/null 2>&1; rm -rf "$worktree"' EXIT
git worktree add --force "$worktree" origin/jazzy >/dev/null 2>&1 || {
  report kilted "SKIP (could not create origin/jazzy worktree — git fetch origin jazzy)"
  exit $failures
}
if docker run --rm -v "$worktree":/ws/src/vector_pursuit_controller:ro \
    ros:kilted-ros-base bash -c '
      apt-get update -qq >/dev/null 2>&1 &&
      apt-get install -y -qq ros-kilted-nav2-core ros-kilted-nav2-util \
        ros-kilted-nav2-costmap-2d ros-kilted-nav2-msgs ros-kilted-nav2-common \
        ros-kilted-nav2-controller ros-kilted-angles \
        ros-kilted-tf2-geometry-msgs >/dev/null 2>&1 &&
      source /opt/ros/kilted/setup.bash && cd /ws &&
      colcon build --packages-select vector_pursuit_controller \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF >/dev/null 2>&1 &&
      test -f install/vector_pursuit_controller/lib/libvector_pursuit_controller.so' \
    >/dev/null 2>&1
then
  report kilted "PASS (source build of jazzy branch)"
else
  report kilted "FAIL (source build of jazzy branch)"
  failures=$((failures + 1))
fi

echo "[check] lyrical: covered by ./docker/build.sh (run separately)"
exit $failures
