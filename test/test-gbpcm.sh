#!/usr/bin/env bash

set -eu
set -o pipefail

export PATH="../bin:$PATH"

mkdir -p testdata/results

gbpcm \
--fixed testdata/pcfix.xyz \
--movable testdata/pcmov.xyz \
--transform testdata/results/pcmov.gbpcm \
--voxel_size 25 \
--grid_limits 24880,354170,110,25955,354595,235 \
--buffer_voxels 1 \
--num_iterations 5 \
--weights "0.1,0.1,0.1,0.1"

gbpcm-transform \
--pc_in testdata/pcmov.xyz \
--pc_out testdata/results/pcmov_transformed.xyz \
--transform testdata/results/pcmov.gbpcm