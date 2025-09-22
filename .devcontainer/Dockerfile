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
    flex \
    wget \
    curl

# Install miniconda for PDAL
# Note: Installing PDAL with vcpkg is possible, but it builds from source and is very slow.
# Using the precompiled conda package is much faster.
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    chmod +x /tmp/miniconda.sh && \
    /tmp/miniconda.sh -b -p /opt/miniconda && \
    rm /tmp/miniconda.sh

# Add conda to PATH for all users
ENV PATH="/opt/miniconda/bin:$PATH"

# Configure conda
RUN conda config --remove channels defaults && \
    conda config --add channels conda-forge && \
    conda config --set channel_priority strict

# Accept Conda Terms of Service
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# Install PDAL via conda
RUN conda install pdal -y --quiet && \
    conda clean -a -y --quiet