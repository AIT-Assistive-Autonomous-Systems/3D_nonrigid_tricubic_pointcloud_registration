clc; clear; close;

addrepo('ptCloud');

pathToPCBasename = "/home/philipp/prj/optrals-tests/data-processing/2018_LandNOE-DSM_00000/03_Lieferungen/2021.10.06_tiled/Z0_Temp/adjust/gbpcm/tile_30000.0_355000";

pathToPCFix = pathToPCBasename + "_reference.xyz";
pathToPCMov = pathToPCBasename + ".xyz";
pathToPCMovTransformed = pathToPCBasename + "_transformed.xyz";

pcFix = ptCloud(pathToPCFix, 'Label', 'pcFix');
pcMov = ptCloud(pathToPCMov, 'Label', 'pcMov');
pcMovTransformed = ptCloud(pathToPCMovTransformed, 'Label', 'pcMovTransformed');

pcFix.plot;
pcMov.plot;
pcMovTransformed.plot;