#!/usr/bin/env bash

set -eu
set -o pipefail

cd test-nordbahn

export PATH="../../bin:$PATH"
export LD_LIBRARY_PATH=/usr/local/vcpkg/installed/x64-linux/lib

mkdir -p results

nonrigid-icp \
    --fixed pcfix.las \
    --movable pcmov.las \
    --transform results/pcmov.nricp \
    --voxel_size 25 \
    --grid_limits 24880,354170,110,25955,354595,235 \
    --buffer_voxels 1 \
    --matching_mode nn \
    --num_iterations 5 \
    --weights "0.1,0.1,0.1,0.1" \
    --profiling

nonrigid-icp-transform \
    --pc_in pcmov.las \
    --pc_out results/pcmov_transformed.las \
    --transform results/pcmov.nricp \
    --profiling