#!/usr/bin/env bash

set -eu
set -o pipefail

source utils.sh

cd test-eagle

export PATH="../../bin:$PATH"

mkdir -p results

log "estimate transformation"
gbpcm \
--fixed pcfix.txt \
--movable pcmov.txt \
--transform results/pcmov.gbpcm \
--voxel_size 0.50 \
--buffer_voxels 5 \
--matching_mode id \
--num_iterations 1 \
--weights "0.01,0.01,0.01,0.01" \
--max_euclidean_distance 10.0 \
--num_correspondences 10000 \
--debug_dir "results/debug"

log "apply transformation"
gbpcm-transform \
--pc_in pcmov.txt \
--pc_out results/pcmov_transformed.txt \
--transform results/pcmov.gbpcm