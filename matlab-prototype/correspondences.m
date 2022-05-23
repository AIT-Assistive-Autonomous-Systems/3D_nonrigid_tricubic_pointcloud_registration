classdef correspondences < handle
   
    properties (SetAccess=private, GetAccess=public)
        
        % Point clouds
        pcFix
        pcMov
        
        % Indices of correspondences
        idxCorrFix
        idxCorrMov
        
    end
    
    properties (Dependent)
        
        % Coordinates of all active points
        XFix
        XTMov
        
        % Coordinates of correspondences only
        XCorrFix
        XTCorrMov
        
        % Attributes of correspondences
        noCorr
        euclDistance
        pointToPlaneDistance
        
    end
    
    methods
        
        function obj = correspondences(pcFix, pcMov)
            
            obj.pcFix = pcFix;
            obj.pcMov = pcMov;
            
        end
        
        function match(obj)
            
            pcMovActIdx = find(obj.pcMov.act);
            
            idxNN = knnsearch(obj.XTMov, obj.XFix);
            
            obj.pcFix.A.idxNN = NaN(obj.pcFix.noPoints,1);
            obj.pcFix.A.idxNN(obj.pcFix.act) = pcMovActIdx(idxNN);
            
            obj.idxCorrFix = find(obj.pcFix.act);
            obj.idxCorrMov = obj.pcFix.A.idxNN(obj.pcFix.act);
            
        end
        
        function reject(obj, options)
            
            arguments
                obj
                options.MaxEuclDistance (1,1) double = 1;
                options.SigmaMadCriteria (1,1) logical = true;
            end
            
            % MaxEuclDistance
            if ~isinf(options.MaxEuclDistance)
                logicalReject = obj.euclDistance > options.MaxEuclDistance;
                reject(logicalReject)
            end
            
            % SigmaMadCriteria
            if options.SigmaMadCriteria
                sigmaMad = 1.4826*mad(obj.pointToPlaneDistance,1);
                rangePointToPlaneDistance = median(obj.pointToPlaneDistance) + [-1 1] * 3 * sigmaMad;
                logicalReject = obj.pointToPlaneDistance < rangePointToPlaneDistance(1) | ...
                                obj.pointToPlaneDistance > rangePointToPlaneDistance(2);
                reject(logicalReject)
            end
            
            function reject(logicalReject)
                obj.idxCorrFix = obj.idxCorrFix(~logicalReject);
                obj.idxCorrMov = obj.idxCorrMov(~logicalReject);
            end
            
        end
        
        function plot(obj, options)

            arguments
                obj
                options.Color = [0.466 0.674 0.188];
            end
            
            plot3([obj.XCorrFix(:,1)'; obj.XTCorrMov(:,1)'], ...
                  [obj.XCorrFix(:,2)'; obj.XTCorrMov(:,2)'], ...
                  [obj.XCorrFix(:,3)'; obj.XTCorrMov(:,3)'], ...
                  '-', Color=options.Color);
            
        end
        
        function noCorr = get.noCorr(obj)
            
            noCorr = numel(obj.idxCorrFix);
            
        end
        
        function XFix = get.XFix(obj)
            
            XFix = obj.pcFix.X(obj.pcFix.act,:);
            
        end
        
        function XTMov = get.XTMov(obj)
            
            XTMov = obj.pcMov.XT(obj.pcMov.act,:);
            
        end
        
        function XCorrFix = get.XCorrFix(obj)
            
            XCorrFix = obj.pcFix.X(obj.idxCorrFix,:);
            
        end
        
        function XTCorrMov = get.XTCorrMov(obj)
            
            XTCorrMov = obj.pcMov.XT(obj.idxCorrMov,:);
            
        end
        
        function euclDistance = get.euclDistance(obj)
           
            dX = obj.XCorrFix - obj.XTCorrMov;
            euclDistance = vecnorm(dX', 2)';
            
        end
        
        function pointToPlaneDistance = get.pointToPlaneDistance(obj)
            
            dX = obj.XCorrFix - obj.XTCorrMov;
            pointToPlaneDistance = dot(dX, ...
                [obj.pcFix.A.nx(obj.idxCorrFix) ...
                 obj.pcFix.A.ny(obj.idxCorrFix) ...
                 obj.pcFix.A.nz(obj.idxCorrFix)], 2);
            
        end
        
    end
    
end