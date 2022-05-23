classdef ptCloudOptrals < ptCloud & matlab.mixin.Copyable

    properties (SetAccess = public, GetAccess = public)
    
        xTranslationGrid
        yTranslationGrid
        zTranslationGrid
        
        XT
        
    end
    
    methods

        function initializeTranslationGrids(obj, cellSize, options)
            
            arguments
                obj
                cellSize (1,1) double
                options.Buffer (1,1) double = 0;
                options.GridOrigin (3,1) double = [nan nan nan]';
            end
            
            if all(~isnan(options.GridOrigin))
            
                gridOrigin = options.GridOrigin;
                    
            else
                
                gridOrigin = [floor(obj.lim.min(1)-options.Buffer) ...
                              floor(obj.lim.min(2)-options.Buffer) ...
                              floor(obj.lim.min(3)-options.Buffer)];
                
            end
            
            xNoCells = ceil((obj.lim.max(1)+options.Buffer-gridOrigin(1))/cellSize);
            yNoCells = ceil((obj.lim.max(2)+options.Buffer-gridOrigin(2))/cellSize);
            zNoCells = ceil((obj.lim.max(3)+options.Buffer-gridOrigin(3))/cellSize);
            
            obj.xTranslationGrid = translationGrid(...
                gridOrigin, xNoCells, yNoCells, zNoCells, cellSize);
            obj.yTranslationGrid = translationGrid(...
                gridOrigin, xNoCells, yNoCells, zNoCells, cellSize);
            obj.zTranslationGrid = translationGrid(...
                gridOrigin, xNoCells, yNoCells, zNoCells, cellSize);
            
        end
        
        function addPointsForTranslationVectors(obj, options)
            
            arguments
                obj
                options.dxy = 1;
                options.Class = -1;
            end
            
            % Create new points
            xNew = obj.xTranslationGrid.xLim(1):options.dxy:obj.xTranslationGrid.xLim(2);
            yNew = obj.xTranslationGrid.yLim(1):options.dxy:obj.xTranslationGrid.yLim(2);
            zNew = obj.xTranslationGrid.zLim(1):options.dxy:obj.xTranslationGrid.zLim(2);
            [XNew, YNew, ZNew] = meshgrid(xNew, yNew, zNew);
            noNewPoints = numel(XNew);
            
            % Create attribute structure A for new points
            attributeNames = fields(obj.A);
            for i = 1:numel(attributeNames)
                ANew.(attributeNames{i}) = NaN(noNewPoints,1);
            end
            ANew.class(end-noNewPoints+1:end) = options.Class;
            
            obj.addPoints([XNew(:) YNew(:) ZNew(:)], ANew);
            
        end
        
        function transformByTranslationGrids(obj)
            
            [xCellRef, yCellRef, zCellRef, dxn, dyn, dzn] = ...
                obj.xTranslationGrid.getGridReference(obj.X);
            
            tx = getTranslation(obj.xTranslationGrid);
            ty = getTranslation(obj.yTranslationGrid);
            tz = getTranslation(obj.zTranslationGrid);
            
            obj.XT = obj.X + [tx ty tz];
            
            function t = getTranslation(translationGrid)
            
                f = translationGrid.getValuesOfCellCorners(translationGrid.F, ...
                    xCellRef, yCellRef, zCellRef);
                fx = translationGrid.getValuesOfCellCorners(translationGrid.Fx, ...
                    xCellRef, yCellRef, zCellRef);
                fy = translationGrid.getValuesOfCellCorners(translationGrid.Fy, ...
                    xCellRef, yCellRef, zCellRef);
                fz = translationGrid.getValuesOfCellCorners(translationGrid.Fz, ...
                    xCellRef, yCellRef, zCellRef);
                fxy = translationGrid.getValuesOfCellCorners(translationGrid.Fxy, ...
                    xCellRef, yCellRef, zCellRef);
                fxz = translationGrid.getValuesOfCellCorners(translationGrid.Fxz, ...
                    xCellRef, yCellRef, zCellRef);
                fyz = translationGrid.getValuesOfCellCorners(translationGrid.Fyz, ...
                    xCellRef, yCellRef, zCellRef);
                fxyz = translationGrid.getValuesOfCellCorners(translationGrid.Fxyz, ...
                    xCellRef, yCellRef, zCellRef);

                t = translationGrid.getValue(f, fx, fy, fz, fxy, fxz, fyz, fxyz, dxn, dyn, dzn);
                
            end
            
        end
        
    end
    
end