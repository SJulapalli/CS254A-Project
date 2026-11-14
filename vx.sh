#!/usr/bin/env bash
# Run the Vortex toolchain in Docker on an Apple Silicon Mac.
#
# The vortex/ submodule is mounted at /work/vortex; builds go to vortex/build
# (out-of-tree, ignored by vortex's .gitignore). This repo's root is mounted
# at /work/project so our own apps are reachable from inside the container.
#
#   ./vx.sh image        build the Docker image (one time, slow)
#   ./vx.sh setup        configure + build vortex/build (one time, and after
#                        pulling or editing VX_config.toml / Makefiles)
#   ./vx.sh shell        interactive shell in vortex/build
#   ./vx.sh run <cmd>    run one command in vortex/build, e.g.
#                        ./vx.sh run ./ci/blackbox.sh --driver=simx --app=vecadd

set -e

PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=vortex-cs254a
SRC="$PROJ_DIR/vortex"
MOUNT=/work/vortex
WORKDIR=$MOUNT/build
CCACHE_VOL=vortex-ccache

SETUP_CMD="mkdir -p build && cd build && ../configure --tooldir=/opt \
  && make -s -C ../third_party && make -s -C sw kernel && make -s -C sw/runtime stub simx"

docker_run() {
    # MAKEFLAGS: compile in parallel (emulated x86 is slow).
    # ccache volume: keeps the compiler cache across container runs.
    docker run --rm --platform linux/amd64 \
        -e MAKEFLAGS="-j$(docker info --format "{{.NCPU}}" 2>/dev/null || echo 4)" \
        -e CCACHE_DIR=/ccache -v "$CCACHE_VOL:/ccache" \
        -v "$SRC:$MOUNT" \
        -v "$PROJ_DIR:/work/project" \
        -w "$WORKDIR" \
        "$@"
}

case "$1" in
    image)
        source "$SRC/VERSION"
        docker build --platform linux/amd64 \
            --build-arg TOOLCHAIN_REV="$TOOLCHAIN_REV" \
            -t "$IMAGE" -f "$PROJ_DIR/docker/Dockerfile" "$SRC/ci"
        ;;
    setup)
        docker_run -w "$MOUNT" "$IMAGE" bash -c "$SETUP_CMD"
        ;;
    shell)
        docker_run -it "$IMAGE" bash
        ;;
    run)
        shift
        docker_run "$IMAGE" bash -c "$*"
        ;;
    *)
        sed -n '2,14p' "$0"
        exit 1
        ;;
esac
