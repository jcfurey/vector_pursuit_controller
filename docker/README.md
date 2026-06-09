# Building `vector_pursuit_controller` for ROS 2 Lyrical

ROS 2 **Lyrical** targets **Ubuntu 26.04 (Resolute)**, and Nav2 has **no binary
packages for Lyrical yet**. This directory provides a one-command, reproducible
build that runs in the official `ros:lyrical-ros-base` image and compiles the
controller's Nav2 dependency closure from source at a **pinned navigation2
commit** (see `NAV2_COMMIT` in the Dockerfile).

## Quick start

From the repository root:

```bash
./docker/build.sh
```

This builds the image `vpc-lyrical`, verifies that the controller loads as a
`nav2_core::Controller` plugin, and verifies no extra CA certificates leaked
into the image. Open a shell with the workspace sourced:

```bash
docker run --rm -it vpc-lyrical bash
# inside the container:
ros2 pkg prefix vector_pursuit_controller
```

To build the image directly:

```bash
docker build -f docker/Dockerfile -t vpc-lyrical .
```

Bump the pinned Nav2 with `--build-arg NAV2_COMMIT=<sha>` once a newer
navigation2 commit is known to build.

## What the build does

1. Clones `ros-navigation/navigation2` at the pinned `NAV2_COMMIT` — Lyrical
   has no Nav2 binaries.
2. Installs the dependency closure with rosdep (scoped to the packages that
   are actually built), keyed on `package.xml` only.
3. Applies two workarounds (`docker/patches/apply_patches.sh`) — after rosdep,
   because one patch targets the BehaviorTree.CPP headers rosdep installs.
4. Builds the 11 nav2 dependency packages.
5. Copies the full controller source and builds just the controller.

Because steps 1–4 are keyed on the pin and the manifest, editing controller
source only re-runs step 5 (about a minute) instead of the full ~20 minute
build.

## The two workarounds (and why)

These cover skew between bleeding-edge Nav2 `main` and Lyrical's shipped
toolchain. Both should disappear once Nav2 cuts a Lyrical release. Each patch
verifies its edit landed and fails the build loudly if upstream drifted.

- **BehaviorTree.CPP getter** — Lyrical ships BT.CPP `4.9.0`, which lacks the
  public `Tree::wakeUpSignal()` getter that Nav2 `main` now calls. The patch
  adds the inline accessor exactly as BT.CPP `master` defines it (it returns an
  already-present private member). Note: an `apt upgrade` inside a container
  reverts this; re-run `apply_patches.sh` before rebuilding in that case.
- **GCC 15 warnings-as-errors** — Ubuntu 26.04's GCC 15 turns four warning
  categories into errors under nav2's `-Werror` (`deprecated-declarations`,
  `free-nonheap-object`, `null-dereference`, `maybe-uninitialized` — the full
  set observed in a clean build of the closure). The patch demotes exactly
  those categories with scoped `-Wno-error=` flags; all other warnings remain
  fatal.

## CA certificates (proxied networks)

If the host trusts extra CAs (corporate / sandbox egress proxy), `build.sh`
passes `/usr/local/share/ca-certificates` as a **build-only context**
(`--build-context hostcerts=...`). Network-touching build steps merge those
certs into a temporary bundle via `docker/with-host-ca.sh` that is created and
removed within the same step, so the certs are **never written to an image
layer** — the shipped image's trust store is byte-identical to the base
image's, and `build.sh` verifies that after every build. On an unrestricted
network this is all a no-op.

For a direct `docker build` behind a proxy, add the flag yourself:

```bash
docker build --build-context hostcerts=/usr/local/share/ca-certificates \
  -f docker/Dockerfile -t vpc-lyrical .
```
