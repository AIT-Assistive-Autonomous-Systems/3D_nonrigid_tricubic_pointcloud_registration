% Run this after run_optrals_adjust_3D_prototype.m

figure('Color', 'k');

xslice = et.pcMov.xTranslationGrid.xLim(1):20*et.pcMov.xTranslationGrid.cellSize:et.pcMov.xTranslationGrid.xLim(2);
yslice = et.pcMov.xTranslationGrid.yLim(1)+25*et.pcMov.xTranslationGrid.cellSize;
zslice = et.pcMov.xTranslationGrid.zLim(1)+5*et.pcMov.xTranslationGrid.cellSize;

tiledlayout(3,1);

nexttile;
slice(et.pcMov.xTranslationGrid.X, et.pcMov.xTranslationGrid.Y, et.pcMov.xTranslationGrid.Z, ...
    pagetranspose(et.pcMov.xTranslationGrid.F), xslice, yslice, zslice);
axis equal
colormap(difpal)
colorbar;
caxis(0.24*[-1 1]);
title('xTranslationGrid');
ct.setdarkmode;

nexttile;
slice(et.pcMov.xTranslationGrid.X, et.pcMov.xTranslationGrid.Y, et.pcMov.xTranslationGrid.Z, ...
    pagetranspose(et.pcMov.yTranslationGrid.F), xslice, yslice, zslice);
axis equal
colormap(difpal)
colorbar;
caxis(0.24*[-1 1]);
title('yTranslationGrid');
ct.setdarkmode;

nexttile;
slice(et.pcMov.xTranslationGrid.X, et.pcMov.xTranslationGrid.Y, et.pcMov.xTranslationGrid.Z, ...
    pagetranspose(et.pcMov.zTranslationGrid.F), xslice, yslice, zslice);
axis equal
colormap(difpal)
colorbar;
caxis(0.24*[-1 1]);
title('zTranslationGrid');
ct.setdarkmode;