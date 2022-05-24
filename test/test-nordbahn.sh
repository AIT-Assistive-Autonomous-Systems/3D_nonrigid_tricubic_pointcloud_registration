#!/usr/bin/env bash

set -eu
set -o pipefail

source utils.sh

cd test-nordbahn

export PATH="../../bin:$PATH"

mkdir -p results

log "estimate transformation"
gbpcm \
--fixed pcfix.xyz \
--movable pcmov.xyz \
--transform results/pcmov.gbpcm \
--voxel_size 25 \
--grid_limits 24880,354170,110,25955,354595,235 \
--buffer_voxels 1 \
--num_iterations 5 \
--weights "0.1,0.1,0.1,0.1"

log "apply transformation"
gbpcm-transform \
--pc_in pcmov.xyz \
--pc_out results/pcmov_transformed.xyz \
--transform results/pcmov.gbpcm