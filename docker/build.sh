#!/usr/bin/env bash
# One-command reproducible ROS 2 Lyrical build of vector_pursuit_controller.
#
# Builds the Docker image (Nav2 from source at a pinned commit + this package),
# then verifies the controller loads as a nav2_core plugin and that no extra
# CA certificates leaked into the image. If the host trusts extra CAs (e.g. a
# corporate or sandbox egress proxy), they are passed as a build-only context
# so apt/git/rosdep can fetch over TLS — they are never baked into the image.
#
#     ./docker/build.sh
#
set -euo pipefail
cd "$(dirname "$0")/.."          # repository root
IMAGE="${IMAGE:-vpc-lyrical}"

ctx_args=()
if compgen -G "/usr/local/share/ca-certificates/*.crt" >/dev/null; then
  ctx_args+=(--build-context hostcerts=/usr/local/share/ca-certificates)
  echo "[build] host CA certs found; passing them as a build-only context"
fi

echo "[build] docker build -f docker/Dockerfile -t ${IMAGE} ."
docker build "${ctx_args[@]}" -f docker/Dockerfile -t "${IMAGE}" .

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

echo "[verify] confirming no extra CA certs were baked into the image..."
base_cas=$(docker run --rm ros:lyrical-ros-base grep -c "BEGIN CERTIFICATE" /etc/ssl/certs/ca-certificates.crt)
image_cas=$(docker run --rm "${IMAGE}" grep -c "BEGIN CERTIFICATE" /etc/ssl/certs/ca-certificates.crt)
if [[ "${image_cas}" -gt "${base_cas}" ]]; then
  echo "  ERROR: image trust store has ${image_cas} certs vs ${base_cas} in the base image" >&2
  exit 1
fi
echo "  OK: image trust store matches the base image (${image_cas} certs)"

echo "[done] image '${IMAGE}' ready. Open a shell with:"
echo "  docker run --rm -it ${IMAGE} bash"
