# Building `vector_pursuit_controller` for ROS 2 Lyrical

ROS 2 **Lyrical** targets **Ubuntu 26.04 (Resolute)**, and Nav2 has **no binary
packages for Lyrical yet**. This directory provides a one-command, reproducible
build that runs in the official `ros:lyrical-ros-base` image and compiles the
controller's Nav2 dependency closure from the `navigation2` `main` branch.

## Quick start

From the repository root:

```bash
./docker/build.sh
```

This builds the image `vpc-lyrical` and verifies that the controller loads as a
`nav2_core::Controller` plugin. Open a shell with the workspace sourced:

```bash
docker run --rm -it vpc-lyrical bash
# inside the container:
ros2 pkg prefix vector_pursuit_controller
```

To build the image directly (without the CA/verify wrapper):

```bash
docker build -f docker/Dockerfile -t vpc-lyrical .
```

Override the Nav2 branch with `--build-arg NAV2_BRANCH=<branch>`.

## What the build does

1. Clones `ros-navigation/navigation2` (`main`) — Lyrical has no Nav2 binaries.
2. Copies this package in alongside it.
3. Applies two workarounds (`docker/patches/apply_patches.sh`).
4. `rosdep install` + `colcon build --packages-up-to vector_pursuit_controller`.

## The two workarounds (and why)

These cover skew between bleeding-edge Nav2 `main` and Lyrical's shipped
toolchain. Both should disappear once Nav2 cuts a Lyrical release.

- **BehaviorTree.CPP getter** — Lyrical ships BT.CPP `4.9.0`, which lacks the
  public `Tree::wakeUpSignal()` getter that Nav2 `main` now calls. The patch
  adds the inline accessor exactly as BT.CPP `master` defines it (it returns an
  already-present private member).
- **GCC 15 `-Werror`** — Ubuntu 26.04's GCC 15 raises a spurious
  `-Werror=null-dereference` in generated rosidl message code. Nav2 builds with
  `-Werror`, so the patch appends `-Wno-error` in `nav2_common`'s
  `nav2_package.cmake` (warnings stay visible, just not fatal).

## CA certificates

If the host trusts extra CAs (corporate / sandbox egress proxy), `build.sh`
copies `/usr/local/share/ca-certificates/*.crt` into `docker/certs/` so the
build can fetch over TLS, then cleans them up. On an unrestricted network this
is a no-op. `*.crt` files in `docker/certs/` are git-ignored.
