# Reproducible ROS 2 Lyrical build of vector_pursuit_controller.
#
# ROS 2 Lyrical targets Ubuntu 26.04 (Resolute) and has no Nav2 binaries yet,
# so this image builds the controller's Nav2 dependency closure from the
# navigation2 'main' branch. Two small workarounds are applied for skew between
# nav2 main and Lyrical's shipped toolchain (see docker/patches/apply_patches.sh).
#
# Build from the repository root:
#     docker build -f docker/Dockerfile -t vpc-lyrical .
# Or use docker/build.sh for automatic proxy-CA handling + verification.

FROM ros:lyrical-ros-base
SHELL ["/bin/bash", "-c"]

# Optional: trust extra CA certificates (corporate / sandbox egress proxy).
# Drop *.crt files into docker/certs/ before building; harmless when empty.
COPY docker/certs/ /usr/local/share/ca-certificates/extra/
RUN update-ca-certificates >/dev/null 2>&1 || true
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt \
    REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt \
    CURL_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt \
    GIT_SSL_CAINFO=/etc/ssl/certs/ca-certificates.crt

ARG NAV2_BRANCH=main
WORKDIR /opt/ws

# Nav2 source (no Lyrical binaries yet) + this package
RUN git clone --depth 1 -b ${NAV2_BRANCH} \
      https://github.com/ros-navigation/navigation2.git src/navigation2
COPY . src/vector_pursuit_controller

# Workarounds for nav2-main-vs-Lyrical skew (BT.CPP getter, GCC 15 -Werror)
RUN bash src/vector_pursuit_controller/docker/patches/apply_patches.sh /opt/ws/src

# Resolve dependencies and build only the controller's dependency closure
RUN source /opt/ros/lyrical/setup.bash && \
    apt-get update -qq && \
    rosdep update --rosdistro lyrical 2>&1 | tail -2 && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical && \
    colcon build --packages-up-to vector_pursuit_controller \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log /var/lib/apt/lists/*

# Auto-source ROS + the overlay for interactive shells
RUN echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ws/install/setup.bash" >> /etc/bash.bashrc
