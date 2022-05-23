clc; clear; close;

paths.in.pcFix = '../gbpcm/testfiles/pcfix.xyz';
paths.in.pcMov = '../gbpcm/testfiles/pcmov.xyz';

paths.in.pcFixOriginal = ...
    '/home/philipp/prj/optrals/data-processing/single-tiles-from-all-datasets/silberwald/01_ETCS_dsm/Tile_24936.0_353546.sbf';
paths.in.pcMovOriginal = ...
    '/home/philipp/prj/optrals/data-processing/single-tiles-from-all-datasets/silberwald/02_OBB_dtm/Tile_24936.0_353546.sbf';

classIDOriginalPoints = 1;

noIterations = 5;

% -----

addrepo('ptCloud');
addrepo('common-tools');
addrepo('lsAdj');

% -----

ct.logging('Start', StartTimer=true);

ct.logging('Read point clouds from files');

pcFix = ptCloudOptrals(paths.in.pcFix, 'Attributes', {'nx' 'ny' 'nz'});
pcFix.A.class = classIDOriginalPoints*ones(pcFix.noPoints, 1);

pcMov = ptCloudOptrals(paths.in.pcMov, 'Attributes', {'nx' 'ny' 'nz'});
pcMov.A.class = classIDOriginalPoints*ones(pcMov.noPoints, 1);

ct.logging('Define translation grids (x/y/z) for movable point cloud');
pcMov.initializeTranslationGrids(25, Buffer=50);

ct.logging('Create optimization object');
et = estimateTrafo(pcFix, pcMov);

% ct.logging('Filter points');
% et.filterPoints(MinPlanarity=0.90);

ct.logging('Selection of correspondences');
% et.selectCorrespondencesByUniformSampling(5);
et.selectCorrespondencesByIntervalSampling(30);

for idxIt = 1:noIterations

    pcMov.transformByTranslationGrids;
    
    et.corr.match;
    et.corr.reject(...
        MaxEuclDistance=1, ...
        SigmaMadCriteria=true);
    
    loggingOfCorrespondences(idxIt, et)
    
    et.adjustmentOptimizationToolbox(...
        WeightZeroObsF=1, ...
        WeightZeroObsFxFyFz=1, ...
        WeightZeroObsFxyFxzFyz=1, ...
        WeightZeroObsFxyz=1);

%     et.adjustmentLsAdj(...
%         WeightZeroObsF=0.5, ...
%         WeightZeroObsFxFyFz=0.5, ...
%         WeightZeroObsFxyFxzFyz=0.5, ...
%         WeightZeroObsFxyz=0.5);
    
end

writematrix(pcMov.XT, 'debug_XT.csv');

% ct.logging('Add points for translation vectors');
% pcMov.addPointsForTranslationVectors(dxy=25);

% ct.logging('Final transformation of points');
% pcMov.transformByTranslationGrids;

ct.logging('Finished');

function loggingOfCorrespondences(idxIt, et)

    if idxIt == 1
        header = sprintf('%10s %10s %10s %10s %10s', ...
            'idxIt', ...
            'noCorr', ...
            'mean(dp)', ...
            'std(dp)');
        ct.logging(header);
    end
    
    stats = sprintf('%10d %10d %10.3f %10.3f', ...
        idxIt, ...
        et.corr.noCorr, ...
        mean(et.corr.pointToPlaneDistance), ...
        std(et.corr.pointToPlaneDistance));
    ct.logging(stats);

end