classdef estimateTrafo < handle

    properties (SetAccess=private, GetAccess=public)
        
        pcFix
        pcMov
        
        corr
        
    end
    
    methods
        
        function obj = estimateTrafo(pcFix, pcMov)
            
            arguments
                pcFix {mustBeA(pcFix, 'ptCloud')}
                pcMov {mustBeA(pcMov, 'ptCloud')}
            end
            
            obj.pcFix = pcFix;
            obj.pcMov = pcMov;
            
            obj.corr = correspondences(pcFix, pcMov);
        
        end
        
        function filterPoints(obj, options)
            
            arguments
                obj
                options.MinPlanarity (1,1) double = 0.97;
            end
            
            assert(isfield(obj.pcFix.A, 'Planarity'));
            assert(isfield(obj.pcMov.A, 'Planarity'));
            
            % Deselect points without normals
            obj.pcFix.act(isnan(obj.pcFix.A.nx)) = false;
            obj.pcMov.act(isnan(obj.pcMov.A.nx)) = false;
            
            % Deselect non-planar points
            obj.pcFix.act(obj.pcFix.A.Planarity <= options.MinPlanarity) = false;
            obj.pcMov.act(obj.pcMov.A.Planarity <= options.MinPlanarity) = false;
            
        end
        
        function selectCorrespondencesByUniformSampling(obj, voxelSize)
            
            arguments
                obj
                voxelSize (1,1) double = 1;
            end
            
            obj.pcFix.select('UniformSampling', voxelSize);
            
        end
        
        function selectCorrespondencesByIntervalSampling(obj, n)
            
            arguments
                obj
                n (1,1) double = 1;
            end
            
            obj.pcFix.select('IntervalSampling', n);
            
        end
        
        function adjustmentLsAdj(obj, options)
            
            arguments
                obj
                options.WeightZeroObsF = 1;
                options.WeightZeroObsFxFyFz = 1;
                options.WeightZeroObsFxyFxzFyz = 1;
                options.WeightZeroObsFxyz = 1;
            end
            
            lsa = lsAdj;
            
            % Add parameters
            for xyz = ["x" "y" "z"]
                for val = ["F" "Fx" "Fy" "Fz" "Fxy" "Fxz" "Fyz" "Fxyz"]
                    
                    if strlength(val) == 1
                        sigFicObs = 1/options.WeightZeroObsF;
                    elseif strlength(val) == 2
                        sigFicObs = 1/options.WeightZeroObsFxFyFz;
                    elseif strlength(val) == 3
                        sigFicObs = 1/options.WeightZeroObsFxyFxzFyz;
                    elseif strlength(val) == 4
                        sigFicObs = 1/options.WeightZeroObsFxyz;
                    end
                        
                    fieldname = xyz + "TranslationGrid";
                    
                    lsa.U.idxPrm.(fieldname + val) = lsa.prm.add(...
                        'x0', obj.pcMov.(fieldname).(val)(:), ...
                        'label', {fieldname + "." + val}, ...
                        'estimateSig', false, ...
                        'sigFicObs', sigFicObs);
                    
                end
            end

            % Add constants
            lsa.U.idxCst.x1 = lsa.cst.add(obj.corr.pcFix.X(obj.corr.idxCorrFix,1));
            lsa.U.idxCst.y1 = lsa.cst.add(obj.corr.pcFix.X(obj.corr.idxCorrFix,2));
            lsa.U.idxCst.z1 = lsa.cst.add(obj.corr.pcFix.X(obj.corr.idxCorrFix,3));
            lsa.U.idxCst.x2 = lsa.cst.add(obj.corr.pcMov.X(obj.corr.idxCorrMov,1));
            lsa.U.idxCst.y2 = lsa.cst.add(obj.corr.pcMov.X(obj.corr.idxCorrMov,2));
            lsa.U.idxCst.z2 = lsa.cst.add(obj.corr.pcMov.X(obj.corr.idxCorrMov,3));
            lsa.U.idxCst.nx = lsa.cst.add(obj.corr.pcFix.A.nx(obj.corr.idxCorrFix));
            lsa.U.idxCst.ny = lsa.cst.add(obj.corr.pcFix.A.ny(obj.corr.idxCorrFix));
            lsa.U.idxCst.nz = lsa.cst.add(obj.corr.pcFix.A.nz(obj.corr.idxCorrFix));
            
            [xCellRef, yCellRef, zCellRef, dxn, dyn, dzn] = ...
                obj.pcMov.xTranslationGrid.getGridReference(...
                obj.corr.pcMov.X(obj.corr.idxCorrMov,:));
            
            lsa.U.idxCst.xCellRef = lsa.cst.add(xCellRef);
            lsa.U.idxCst.yCellRef = lsa.cst.add(yCellRef);
            lsa.U.idxCst.zCellRef = lsa.cst.add(zCellRef);
            lsa.U.idxCst.dxn = lsa.cst.add(dxn);
            lsa.U.idxCst.dyn = lsa.cst.add(dyn);
            lsa.U.idxCst.dzn = lsa.cst.add(dzn);
            
            lsa.U.idxCst.xNoCells = lsa.cst.add(obj.pcMov.xTranslationGrid.xNoCells);
            lsa.U.idxCst.yNoCells = lsa.cst.add(obj.pcMov.xTranslationGrid.yNoCells);
            lsa.U.idxCst.zNoCells = lsa.cst.add(obj.pcMov.xTranslationGrid.zNoCells);
            
            % Add observations
            lsa.U.idxObs.dp = lsa.obs.add(...
                'b', zeros(obj.corr.noCorr,1), ...
                'sig', ones(obj.corr.noCorr,1));
            
            % Add conditions
            % Aktuell noch falsch: wir gehen hier mit allen prm und cst rein statt nur mit jenen die
            % sich auf die obs beziehen!
            % Hier muss also irgendwo sub2ind angewandt werden, dann geht man mit einem Vektor in
            % die Beob.gl. rein.
            lsa.con.add('fcn', @conPointToPlaneDistance, ...
                'prm', lsa.U.idxPrm, ...
                'cst', lsa.U.idxCst, ...
                'obs', lsa.U.idxObs);
            
            % Solve!
            lsa.solve;
            
        end
        
        function adjustmentOptimizationToolbox(obj, options)
            
            arguments
                obj
                options.WeightZeroObsF = 1;
                options.WeightZeroObsFxFyFz = 1;
                options.WeightZeroObsFxyFxzFyz = 1;
                options.WeightZeroObsFxyz = 1;
            end
            
            problem = optimproblem;
            
            xTranslationGridF    = optimvar('xTranslationGridF'   , size(obj.pcMov.xTranslationGrid.F));
            xTranslationGridFx   = optimvar('xTranslationGridFx'  , size(obj.pcMov.xTranslationGrid.Fx));
            xTranslationGridFy   = optimvar('xTranslationGridFy'  , size(obj.pcMov.xTranslationGrid.Fy));
            xTranslationGridFz   = optimvar('xTranslationGridFz'  , size(obj.pcMov.xTranslationGrid.Fz));
            xTranslationGridFxy  = optimvar('xTranslationGridFxy' , size(obj.pcMov.xTranslationGrid.Fxy));
            xTranslationGridFxz  = optimvar('xTranslationGridFxz' , size(obj.pcMov.xTranslationGrid.Fxz));
            xTranslationGridFyz  = optimvar('xTranslationGridFyz' , size(obj.pcMov.xTranslationGrid.Fyz));
            xTranslationGridFxyz = optimvar('xTranslationGridFxyz', size(obj.pcMov.xTranslationGrid.Fxyz));
            
            yTranslationGridF    = optimvar('yTranslationGridF'   , size(obj.pcMov.yTranslationGrid.F));
            yTranslationGridFx   = optimvar('yTranslationGridFx'  , size(obj.pcMov.yTranslationGrid.Fx));
            yTranslationGridFy   = optimvar('yTranslationGridFy'  , size(obj.pcMov.yTranslationGrid.Fy));
            yTranslationGridFz   = optimvar('yTranslationGridFz'  , size(obj.pcMov.yTranslationGrid.Fz));
            yTranslationGridFxy  = optimvar('yTranslationGridFxy' , size(obj.pcMov.yTranslationGrid.Fxy));
            yTranslationGridFxz  = optimvar('yTranslationGridFxz' , size(obj.pcMov.yTranslationGrid.Fxz));
            yTranslationGridFyz  = optimvar('yTranslationGridFyz' , size(obj.pcMov.yTranslationGrid.Fyz));
            yTranslationGridFxyz = optimvar('yTranslationGridFxyz', size(obj.pcMov.yTranslationGrid.Fxyz));
            
            zTranslationGridF    = optimvar('zTranslationGridF'   , size(obj.pcMov.zTranslationGrid.F));
            zTranslationGridFx   = optimvar('zTranslationGridFx'  , size(obj.pcMov.zTranslationGrid.Fx));
            zTranslationGridFy   = optimvar('zTranslationGridFy'  , size(obj.pcMov.zTranslationGrid.Fy));
            zTranslationGridFz   = optimvar('zTranslationGridFz'  , size(obj.pcMov.zTranslationGrid.Fz));
            zTranslationGridFxy  = optimvar('zTranslationGridFxy' , size(obj.pcMov.zTranslationGrid.Fxy));
            zTranslationGridFxz  = optimvar('zTranslationGridFxz' , size(obj.pcMov.zTranslationGrid.Fxz));
            zTranslationGridFyz  = optimvar('zTranslationGridFyz' , size(obj.pcMov.zTranslationGrid.Fyz));
            zTranslationGridFxyz = optimvar('zTranslationGridFxyz', size(obj.pcMov.zTranslationGrid.Fxyz));
            
            x0.xTranslationGridF    = obj.pcMov.xTranslationGrid.F;
            x0.xTranslationGridFx   = obj.pcMov.xTranslationGrid.Fx;
            x0.xTranslationGridFy   = obj.pcMov.xTranslationGrid.Fy;
            x0.xTranslationGridFz   = obj.pcMov.xTranslationGrid.Fz;
            x0.xTranslationGridFxy  = obj.pcMov.xTranslationGrid.Fxy;
            x0.xTranslationGridFxz  = obj.pcMov.xTranslationGrid.Fxz;
            x0.xTranslationGridFyz  = obj.pcMov.xTranslationGrid.Fyz;
            x0.xTranslationGridFxyz = obj.pcMov.xTranslationGrid.Fxyz;
            
            x0.yTranslationGridF    = obj.pcMov.yTranslationGrid.F;
            x0.yTranslationGridFx   = obj.pcMov.yTranslationGrid.Fx;
            x0.yTranslationGridFy   = obj.pcMov.yTranslationGrid.Fy;
            x0.yTranslationGridFz   = obj.pcMov.yTranslationGrid.Fz;
            x0.yTranslationGridFxy  = obj.pcMov.yTranslationGrid.Fxy;
            x0.yTranslationGridFxz  = obj.pcMov.yTranslationGrid.Fxz;
            x0.yTranslationGridFyz  = obj.pcMov.yTranslationGrid.Fyz;
            x0.yTranslationGridFxyz = obj.pcMov.yTranslationGrid.Fxyz;
            
            x0.zTranslationGridF    = obj.pcMov.zTranslationGrid.F;
            x0.zTranslationGridFx   = obj.pcMov.zTranslationGrid.Fx;
            x0.zTranslationGridFy   = obj.pcMov.zTranslationGrid.Fy;
            x0.zTranslationGridFz   = obj.pcMov.zTranslationGrid.Fz;
            x0.zTranslationGridFxy  = obj.pcMov.zTranslationGrid.Fxy;
            x0.zTranslationGridFxz  = obj.pcMov.zTranslationGrid.Fxz;
            x0.zTranslationGridFyz  = obj.pcMov.zTranslationGrid.Fyz;
            x0.zTranslationGridFxyz = obj.pcMov.zTranslationGrid.Fxyz;
            
            [xCellRef, yCellRef, zCellRef, dxn, dyn, dzn] = ...
                obj.pcMov.xTranslationGrid.getGridReference(...
                obj.corr.pcMov.X(obj.corr.idxCorrMov,:));
            
            tx = getTranslation(...
                    xTranslationGridF, ...
                    xTranslationGridFx, ...
                    xTranslationGridFy, ...
                    xTranslationGridFz, ...
                    xTranslationGridFxy, ...
                    xTranslationGridFxz, ...
                    xTranslationGridFyz, ...
                    xTranslationGridFxyz);
                
            ty = getTranslation(...
                    yTranslationGridF, ...
                    yTranslationGridFx, ...
                    yTranslationGridFy, ...
                    yTranslationGridFz, ...
                    yTranslationGridFxy, ...
                    yTranslationGridFxz, ...
                    yTranslationGridFyz, ...
                    yTranslationGridFxyz);
                
            tz = getTranslation(...
                    zTranslationGridF, ...
                    zTranslationGridFx, ...
                    zTranslationGridFy, ...
                    zTranslationGridFz, ...
                    zTranslationGridFxy, ...
                    zTranslationGridFxz, ...
                    zTranslationGridFyz, ...
                    zTranslationGridFxyz);
            
            function t = getTranslation(...
                    translationGridF, ...
                    translationGridFx, ...
                    translationGridFy, ...
                    translationGridFz, ...
                    translationGridFxy, ...
                    translationGridFxz, ...
                    translationGridFyz, ...
                    translationGridFxyz)
            
                f = translationGrid.getValuesOfCellCorners(translationGridF, ...
                    xCellRef, yCellRef, zCellRef);
                fx = translationGrid.getValuesOfCellCorners(translationGridFx, ...
                    xCellRef, yCellRef, zCellRef);
                fy = translationGrid.getValuesOfCellCorners(translationGridFy, ...
                    xCellRef, yCellRef, zCellRef);
                fz = translationGrid.getValuesOfCellCorners(translationGridFz, ...
                    xCellRef, yCellRef, zCellRef);
                fxy = translationGrid.getValuesOfCellCorners(translationGridFxy, ...
                    xCellRef, yCellRef, zCellRef);
                fxz = translationGrid.getValuesOfCellCorners(translationGridFxz, ...
                    xCellRef, yCellRef, zCellRef);
                fyz = translationGrid.getValuesOfCellCorners(translationGridFyz, ...
                    xCellRef, yCellRef, zCellRef);
                fxyz = translationGrid.getValuesOfCellCorners(translationGridFxyz, ...
                    xCellRef, yCellRef, zCellRef);

                t = translationGrid.getValue(f, fx, fy, fz, fxy, fxz, fyz, fxyz, dxn, dyn, dzn);
                
            end
            
            dX = obj.corr.pcFix.X(obj.corr.idxCorrFix,:) - ...
                 (obj.corr.pcMov.X(obj.corr.idxCorrMov,:) + [tx ty tz]);

            vCorr = dot(dX, ...
                        [obj.corr.pcFix.A.nx(obj.corr.idxCorrFix) ...
                         obj.corr.pcFix.A.ny(obj.corr.idxCorrFix) ...
                         obj.corr.pcFix.A.nz(obj.corr.idxCorrFix)], 2);

            vF = [xTranslationGridF - zeros(size(xTranslationGridF))
                  yTranslationGridF - zeros(size(yTranslationGridF))
                  zTranslationGridF - zeros(size(zTranslationGridF))];

            vFx = [xTranslationGridFx - zeros(size(xTranslationGridFx))
                   yTranslationGridFx - zeros(size(yTranslationGridFx))
                   zTranslationGridFx - zeros(size(zTranslationGridFx))];
               
            vFy = [xTranslationGridFy - zeros(size(xTranslationGridFy))
                   yTranslationGridFy - zeros(size(yTranslationGridFy))
                   zTranslationGridFy - zeros(size(zTranslationGridFy))];
               
            vFz = [xTranslationGridFz - zeros(size(xTranslationGridFz))
                   yTranslationGridFz - zeros(size(yTranslationGridFz))
                   zTranslationGridFz - zeros(size(zTranslationGridFz))];
               
            vFxy = [xTranslationGridFxy - zeros(size(xTranslationGridFxy))
                    yTranslationGridFxy - zeros(size(yTranslationGridFxy))
                    zTranslationGridFxy - zeros(size(zTranslationGridFxy))];

            vFxz = [xTranslationGridFxz - zeros(size(xTranslationGridFxz))
                    yTranslationGridFxz - zeros(size(yTranslationGridFxz))
                    zTranslationGridFxz - zeros(size(zTranslationGridFxz))];

            vFyz = [xTranslationGridFyz - zeros(size(xTranslationGridFyz))
                    yTranslationGridFyz - zeros(size(yTranslationGridFyz))
                    zTranslationGridFyz - zeros(size(zTranslationGridFyz))];

            vFxyz = [xTranslationGridFxyz - zeros(size(xTranslationGridFxyz))
                     yTranslationGridFxyz - zeros(size(yTranslationGridFxyz))
                     zTranslationGridFxyz - zeros(size(zTranslationGridFxyz))];

            v = [vCorr
                 vF(:)*options.WeightZeroObsF
                 vFx(:)*options.WeightZeroObsFxFyFz
                 vFy(:)*options.WeightZeroObsFxFyFz
                 vFz(:)*options.WeightZeroObsFxFyFz
                 vFxy(:)*options.WeightZeroObsFxyFxzFyz
                 vFxz(:)*options.WeightZeroObsFxyFxzFyz
                 vFyz(:)*options.WeightZeroObsFxyFxzFyz
                 vFxyz(:)*options.WeightZeroObsFxyz];
             
            problem.Objective = sum(v.^2);
            
            options = optimoptions('lsqlin');
            options.Display = 'off';
            
            % Solve!
            [solution, vTv] = solve(problem, x0, options=options);
            
%             fprintf('%20s = %.3f\n', 'vTv', vTv);
%             fprintf('%20s = %.3f\n', 'mean(vCorr(x0))', mean(evaluate(vCorr, x0)));
%             fprintf('%20s = %.3f\n', 'std(vCorr(x0))', std(evaluate(vCorr, x0)));
%             fprintf('%20s = %.3f\n', 'mean(vCorr(solution))', mean(evaluate(vCorr, solution)));
%             fprintf('%20s = %.3f\n', 'std(vCorr(solution))', std(evaluate(vCorr, solution)));
            
            vVals = evaluate(v, solution);
            writematrix(vVals, 'optim_v.csv');
            
            % Update grids
            obj.pcMov.xTranslationGrid.F = solution.xTranslationGridF;
            obj.pcMov.xTranslationGrid.Fx = solution.xTranslationGridFx;
            obj.pcMov.xTranslationGrid.Fy = solution.xTranslationGridFy;
            obj.pcMov.xTranslationGrid.Fz = solution.xTranslationGridFz;
            obj.pcMov.xTranslationGrid.Fxy = solution.xTranslationGridFxy;
            obj.pcMov.xTranslationGrid.Fxz = solution.xTranslationGridFxz;
            obj.pcMov.xTranslationGrid.Fyz = solution.xTranslationGridFyz;
            obj.pcMov.xTranslationGrid.Fxyz = solution.xTranslationGridFxyz;
            
            obj.pcMov.yTranslationGrid.F = solution.yTranslationGridF;
            obj.pcMov.yTranslationGrid.Fx = solution.yTranslationGridFx;
            obj.pcMov.yTranslationGrid.Fy = solution.yTranslationGridFy;
            obj.pcMov.yTranslationGrid.Fz = solution.yTranslationGridFz;
            obj.pcMov.yTranslationGrid.Fxy = solution.yTranslationGridFxy;
            obj.pcMov.yTranslationGrid.Fxz = solution.yTranslationGridFxz;
            obj.pcMov.yTranslationGrid.Fyz = solution.yTranslationGridFyz;
            obj.pcMov.yTranslationGrid.Fxyz = solution.yTranslationGridFxyz;
            
            obj.pcMov.zTranslationGrid.F = solution.zTranslationGridF;
            obj.pcMov.zTranslationGrid.Fx = solution.zTranslationGridFx;
            obj.pcMov.zTranslationGrid.Fy = solution.zTranslationGridFy;
            obj.pcMov.zTranslationGrid.Fz = solution.zTranslationGridFz;
            obj.pcMov.zTranslationGrid.Fxy = solution.zTranslationGridFxy;
            obj.pcMov.zTranslationGrid.Fxz = solution.zTranslationGridFxz;
            obj.pcMov.zTranslationGrid.Fyz = solution.zTranslationGridFyz;
            obj.pcMov.zTranslationGrid.Fxyz = solution.zTranslationGridFxyz;

        end

    end
    
end