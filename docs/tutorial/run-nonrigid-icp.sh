#!/usr/bin/env bash

set -eu
set -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

# Matching mode nn
nonrigid-icp-x86_64.AppImage \
    --fixed original-bunny.ply \
    --movable deformed-bunny.ply \
    --voxel_size 2 \
    --max_euclidean_distance 2.0 \
    --matching_mode nn \
    --num_iterations 10 \
    --weights 0.1,0.1,0.1,0.1 \
    --transform transform-grid-nn.nricp

nonrigid-icp-transform-x86_64.AppImage \
    --pc_in deformed-bunny.ply \
    --pc_out aligned-bunny-nn.ply \
    --transform transform-grid-nn.nricp

# Matching mode id
nonrigid-icp-x86_64.AppImage \
    --fixed original-bunny.ply \
    --movable deformed-bunny.ply \
    --voxel_size 2 \
    --max_euclidean_distance 2.0 \
    --matching_mode id \
    --num_iterations 10 \
    --weights 0.1,0.1,0.1,0.1 \
    --transform transform-grid-id.nricp

nonrigid-icp-transform-x86_64.AppImage \
    --pc_in deformed-bunny.ply \
    --pc_out aligned-bunny-id.ply \
    --transform transform-grid-id.nricp

