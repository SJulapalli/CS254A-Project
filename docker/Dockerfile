# Vortex toolchain image for the CS254A project (adapted from the Lab 1 image).
#
# Only the toolchain lives in the image. The vortex/ submodule is mounted into
# the container at runtime (see ../vx.sh), so edits made on the Mac are visible
# inside the container immediately.
#
# Build context is vortex/ci (only its two install scripts are used).
# TOOLCHAIN_REV comes from vortex/VERSION. Verilator is included so rtlsim is
# available once we start changing the RT unit RTL.

FROM --platform=linux/amd64 ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG TOOLCHAIN_REV=v3.0.1
ARG TOOLCHAIN_PARTS="--llvm --riscv32 --libcrt32 --libc32 --verilator"

# Tools the Vortex install scripts assume are present.
# (perl: the verilator front end is a perl script.)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates wget bzip2 git sudo make python3 perl software-properties-common && \
    rm -rf /var/lib/apt/lists/*

COPY install_dependencies.sh toolchain_install.sh.in /tmp/vx/

# System packages (compiler, cmake, boost, ...) needed to build SimX.
RUN bash /tmp/vx/install_dependencies.sh && rm -rf /var/lib/apt/lists/*

# Prebuilt RISC-V toolchain: 32-bit kernels plus verilator.
ENV TOOLDIR=/opt
RUN cd /tmp/vx && \
    TOOLCHAIN_REV=${TOOLCHAIN_REV} OSVERSION=ubuntu/focal \
    bash toolchain_install.sh.in ${TOOLCHAIN_PARTS} > /dev/null && \
    rm -rf /tmp/vx

ENV PATH=/opt/verilator/bin:/opt/llvm-vortex/bin:$PATH
WORKDIR /work/vortex
