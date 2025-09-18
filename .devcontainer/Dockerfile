FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

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

# Install C++ dependencies via vcpkg using vcpkg.json
COPY vcpkg.json /tmp/vcpkg.json
RUN su vscode -c "cd /tmp && ${VCPKG_ROOT}/vcpkg install"