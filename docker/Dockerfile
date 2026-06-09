# Reproducible ROS 2 Lyrical build of vector_pursuit_controller.
#
# ROS 2 Lyrical targets Ubuntu 26.04 (Resolute) and has no Nav2 binaries yet,
# so this image builds the controller's Nav2 dependency closure from source at
# a pinned navigation2 commit. Two small workarounds are applied for skew
# between nav2 main and Lyrical's shipped toolchain (see
# docker/patches/apply_patches.sh).
#
# Build from the repository root (docker/build.sh adds proxy-CA handling and
# post-build verification):
#     ./docker/build.sh
# or directly:
#     docker build -f docker/Dockerfile -t vpc-lyrical .
#
# Layers are split so that editing controller source only rebuilds the final
# stage (~1 min): nav2 clone and the dependency build are keyed on the pinned
# commit and package.xml, not on the full source tree.

# Build-only CA certificates. Empty by default; on a network behind a
# TLS-intercepting proxy, override with:
#     docker build --build-context hostcerts=/usr/local/share/ca-certificates ...
# The certs are only visible inside individual RUN steps (via with-host-ca)
# and are never written to an image layer.
FROM scratch AS hostcerts

FROM ros:lyrical-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY docker/with-host-ca.sh /usr/local/bin/with-host-ca
RUN chmod +x /usr/local/bin/with-host-ca

# Nav2 source, pinned for reproducibility (no Lyrical binaries yet). Bump by
# passing --build-arg NAV2_COMMIT=<sha> once a newer nav2 is known to build.
ARG NAV2_COMMIT=374cd2556640586251b1e49a346f6c8d0cb76224
WORKDIR /opt/ws
RUN --mount=from=hostcerts,target=/hostcerts \
    with-host-ca bash -c "\
      git init -q src/navigation2 && cd src/navigation2 && \
      git remote add origin https://github.com/ros-navigation/navigation2.git && \
      git fetch -q --depth 1 origin ${NAV2_COMMIT} && \
      git checkout -q FETCH_HEAD"

# Dependency layer: keyed on this package's manifest only, so source edits do
# not invalidate it. Resolves and installs deps for the controller's closure,
# applies the workaround patches (after rosdep — one patch targets the
# BehaviorTree.CPP headers rosdep installs), and builds the 11 nav2 deps.
COPY package.xml src/vector_pursuit_controller/package.xml
COPY docker/patches/apply_patches.sh /opt/apply_patches.sh
RUN --mount=from=hostcerts,target=/hostcerts \
    source /opt/ros/lyrical/setup.bash && \
    apt-get update -qq && \
    with-host-ca rosdep update --rosdistro lyrical && \
    PKG_PATHS="$(colcon list --packages-up-to vector_pursuit_controller --paths-only | tr '\n' ' ')" && \
    rosdep install --from-paths ${PKG_PATHS} --ignore-src -r -y --rosdistro lyrical && \
    bash /opt/apply_patches.sh /opt/ws/src && \
    colcon build --packages-up-to vector_pursuit_controller \
      --packages-skip vector_pursuit_controller \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log /var/lib/apt/lists/*

# Controller layer: the only one invalidated by source edits.
COPY . src/vector_pursuit_controller
RUN source /opt/ros/lyrical/setup.bash && \
    colcon build --packages-select vector_pursuit_controller \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# Auto-source ROS + the overlay for interactive shells
RUN echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ws/install/setup.bash" >> /etc/bash.bashrc
