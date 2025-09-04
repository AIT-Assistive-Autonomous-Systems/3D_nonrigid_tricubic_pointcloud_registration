#!/usr/bin/env bash

set -eu
set -o pipefail

cd test-mls-rail

export PATH="../../bin:$PATH"
export LD_LIBRARY_PATH=/usr/local/vcpkg/installed/x64-linux/lib

echo "pcfix: preprocessing"
pdal pipeline pdal-pipeline.json \
    --readers.las.filename="pcfix.laz" \
    --writers.las.filename="pcfix_preprocessed.laz"

echo "pcmov: preprocessing"
pdal pipeline pdal-pipeline.json \
    --readers.las.filename="pcmov.laz" \
    --writers.las.filename="pcmov_preprocessed.laz"

echo "pcfix: translate to xyz"
pdal translate pcfix_preprocessed.laz pcfix_preprocessed.xyz \
    --writers.text.order="X:3,Y:3,Z:3,NormalX:3,NormalY:3,NormalZ:3" \
    --writers.text.write_header="false" \
    --writers.text.delimiter=" " \
    --writers.text.keep_unspecified="false"

echo "pcmov: translate to xyz"
pdal translate pcmov_preprocessed.laz pcmov_preprocessed.xyz \
    --writers.text.order="X:3,Y:3,Z:3,NormalX:3,NormalY:3,NormalZ:3" \
    --writers.text.write_header="false" \
    --writers.text.delimiter=" " \
    --writers.text.keep_unspecified="false"

mkdir -p results

echo "estimate transformation"
nonrigid-icp \
    --fixed pcfix_preprocessed.xyz \
    --movable pcmov_preprocessed.xyz \
    --transform results/pcmov.nricp \
    --voxel_size 20

echo "apply transformation"
nonrigid-icp-transform \
    --pc_in pcmov_preprocessed.xyz \
    --pc_out results/pcmov_preprocessed_transformed.xyz \
    --transform results/pcmov.nricp
