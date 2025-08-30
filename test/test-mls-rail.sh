#!/usr/bin/env bash

set -eu
set -o pipefail

source utils.sh

cd test-mls-rail

export PATH="../../bin:$PATH"

log "pcfix: preprocessing"
pdal pipeline pdal-pipeline.json \
    --readers.las.filename="pcfix.laz" \
    --writers.las.filename="pcfix_preprocessed.laz"

log "pcmov: preprocessing"
pdal pipeline pdal-pipeline.json \
    --readers.las.filename="pcmov.laz" \
    --writers.las.filename="pcmov_preprocessed.laz"

log "pcfix: translate to xyz"
pdal translate pcfix_preprocessed.laz pcfix_preprocessed.xyz \
    --writers.text.order="X:3,Y:3,Z:3,NormalX:3,NormalY:3,NormalZ:3" \
    --writers.text.write_header="false" \
    --writers.text.delimiter=" " \
    --writers.text.keep_unspecified="false"

log "pcmov: translate to xyz"
pdal translate pcmov_preprocessed.laz pcmov_preprocessed.xyz \
    --writers.text.order="X:3,Y:3,Z:3,NormalX:3,NormalY:3,NormalZ:3" \
    --writers.text.write_header="false" \
    --writers.text.delimiter=" " \
    --writers.text.keep_unspecified="false"

mkdir -p results

log "estimate transformation"
nonrigid-icp \
    --fixed pcfix_preprocessed.xyz \
    --movable pcmov_preprocessed.xyz \
    --transform results/pcmov.nricp \
    --voxel_size 20

log "apply transformation"
nonrigid-icp-transform \
    --pc_in pcmov_preprocessed.xyz \
    --pc_out results/pcmov_preprocessed_transformed.xyz \
    --transform results/pcmov.nricp
