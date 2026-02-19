# Virus Module Build Environment
# Targets: Ableton Move (aarch64 Linux, glibc 2.35)
# Uses Ubuntu 22.04 for glibc compatibility with the device.

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    cmake \
    ninja-build \
    make \
    file \
    git \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# Set cross-compilation environment
ENV CROSS_PREFIX=aarch64-linux-gnu-
ENV CC=aarch64-linux-gnu-gcc
ENV CXX=aarch64-linux-gnu-g++
