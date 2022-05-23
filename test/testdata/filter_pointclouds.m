clc; clear; close;

pathIn = '/home/philipp/prj/optrals/data-processing/single-tiles-from-all-datasets/silberwald/01_ETCS_dsm/Tile_24936.0_353546_after-postfilter.sbf';
pathOut = 'pcfix.xyz';
filter(pathIn, pathOut);

pathIn = '/home/philipp/prj/optrals/data-processing/single-tiles-from-all-datasets/silberwald/02_OBB_dtm/Tile_24936.0_353546_after-postfilter.sbf';
pathOut = 'pcmov.xyz';
filter(pathIn, pathOut);

function filter(pathIn, pathOut)

    pc = ptCloud(pathIn);
    pc.normals(1);
    pc.act(isnan(pc.A.nx)) = false;
    pc.select('Attribute', 'Planarity', [0.9 Inf]);
    pc.info;
    pc.export(pathOut, 'Attributes', {'nx' 'ny' 'nz'}, 'Header', false);

end