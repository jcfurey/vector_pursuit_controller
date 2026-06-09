#!/usr/bin/env bash
# One-command reproducible ROS 2 Lyrical build of vector_pursuit_controller.
#
# Builds the Docker image (Nav2 from source + this package), then verifies the
# controller loads as a nav2_core plugin. If the host trusts extra CA certs
# (e.g. a corporate or sandbox egress proxy), they are passed into the build
# automatically so apt/git/rosdep can fetch over TLS.
#
#     ./docker/build.sh
#
set -euo pipefail
cd "$(dirname "$0")/.."          # repository root
IMAGE="${IMAGE:-vpc-lyrical}"

mkdir -p docker/certs
copied=0
if compgen -G "/usr/local/share/ca-certificates/*.crt" >/dev/null; then
  cp /usr/local/share/ca-certificates/*.crt docker/certs/ 2>/dev/null || true
  copied=$(ls docker/certs/*.crt 2>/dev/null | wc -l)
  echo "[build] bundled ${copied} host CA cert(s) for the build"
fi
# Always clean up copied certs from the working tree on exit.
trap 'rm -f docker/certs/*.crt' EXIT

echo "[build] docker build -f docker/Dockerfile -t ${IMAGE} ."
docker build -f docker/Dockerfile -t "${IMAGE}" .

echo "[verify] confirming the controller is a loadable nav2_core plugin..."
docker run --rm "${IMAGE}" bash -lc '
  set -e
  source /opt/ros/lyrical/setup.bash
  source /opt/ws/install/setup.bash
  echo "  prefix: $(ros2 pkg prefix vector_pursuit_controller)"
  nm -DC /opt/ws/install/vector_pursuit_controller/lib/libvector_pursuit_controller.so \
    | grep -q "registerPlugin<vector_pursuit_controller::VectorPursuitController" \
    && echo "  OK: VectorPursuitController registered as nav2_core::Controller"
'
echo "[done] image '${IMAGE}' ready. Open a shell with:"
echo "  docker run --rm -it ${IMAGE} bash"
