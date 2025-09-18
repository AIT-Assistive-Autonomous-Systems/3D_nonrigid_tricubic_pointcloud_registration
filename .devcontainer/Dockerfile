FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

# Install system packages via apt
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    libtool \
    autoconf \
    automake \
    git-lfs \
    pkg-config \
    ninja-build \
    bison \
    flex

# Install C++ dependencies via vcpkg
RUN su vscode -c "${VCPKG_ROOT}/vcpkg install eigen3 spdlog nanoflann pdal cxxopts"