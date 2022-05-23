classdef translationGrid
    
    properties
        
        % Tensor with translation values
        F
        
        % Tensor with derivatives of translation values
        Fx
        Fy
        Fz
        Fxy
        Fxz
        Fyz
        Fxyz
        
        gridOrigin
        cellSize
        
        % First z level of tensors:
        % +-------+-------+-------+-------+
        % |       |       |       |       |
        % |       |       |       |       |
        % |5,1,1  |5,2,1  |5,3,1  |5,4,1  |
        % +-------+-------+-------+-------+
        % |       |       |       |       |
        % |       |       |       |       |
        % |4,1,1  |4,2,1  |4,3,1  |4,4,1  |
        % +-------+-------+-------+-------+
        % |       |       |       |       |
        % |       |       |       |       |
        % |3,1,1  |3,2,1  |3,3,1  |3,4,1  |
        % +-------+-------+-------+-------+
        % |       |       |       |       |
        % |       |       |       |       |
        % |2,1,1  |2,2,1  |2,3,1  |2,4,1  |
        % +-------+-------+-------+-------+
        % |       |       |       |       |
        % |       |       |       |       |
        % |1,1,1  |1,2,1  |1,3,1  |1,4,1  |
        % o-------+-------+-------+-------+
        %
        % o     ... obj.gridOrigin (= lower, left corner of tensor, first z level)
        % x,x,1 ... indices of tensors for first z level, e.g. obj.F
        
    end
    
    properties (Dependent)

        xNoCells
        yNoCells
        zNoCells
        
        X
        Y
        Z
        
        % Extents of tensor boundaries
        xLim
        yLim
        zLim
        
    end
    
    methods
        
        function obj = translationGrid(gridOrigin, xNoCells, yNoCells, zNoCells, cellSize)

            obj.F    = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fx   = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fy   = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fz   = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fxy  = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fxz  = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fyz  = zeros(xNoCells, yNoCells, zNoCells);
            obj.Fxyz = zeros(xNoCells, yNoCells, zNoCells);
            
            obj.gridOrigin = gridOrigin;
            obj.cellSize = cellSize;
            
        end
        
        function [xCellRef, yCellRef, zCellRef, dxn, dyn, dzn] = getGridReference(obj, X)
            
            % Todo Error if outside of xLim/yLim
            
            arguments
                obj
                X (:,3)
            end
            
            % +----------+
            % |   (x/y/z)|
            % |   +      |
            % |          |
            % |          |
            % +----------+
            % reference point is lower, bottom left corner
            
            xGrid = X(:,1)-obj.gridOrigin(1);
            yGrid = X(:,2)-obj.gridOrigin(2);
            zGrid = X(:,3)-obj.gridOrigin(3);
            
            xCellRef = ceil(xGrid/obj.cellSize);
            yCellRef = ceil(yGrid/obj.cellSize);
            zCellRef = ceil(zGrid/obj.cellSize);
            
            % Special handling for points exactly on lower, bottom, left boundary of grid
            xCellRef(X(:,1) == obj.xLim(1)) = 1;
            yCellRef(X(:,2) == obj.yLim(1)) = 1;
            zCellRef(X(:,3) == obj.zLim(1)) = 1;
            
            % Cell coordinates
            dx = xGrid - (xCellRef-1)*obj.cellSize;
            dy = yGrid - (yCellRef-1)*obj.cellSize;
            dz = zGrid - (zCellRef-1)*obj.cellSize;
            
            % Normalized cell coordinates, i.e. from 0 to 1
            dxn = dx/obj.cellSize;
            dyn = dy/obj.cellSize;
            dzn = dz/obj.cellSize;
            
        end
        
        function xNoCells = get.xNoCells(obj)
           
            xNoCells = size(obj.F,1);
            
        end
        
        function yNoCells = get.yNoCells(obj)
           
            yNoCells = size(obj.F,2);
            
        end
        
        function zNoCells = get.zNoCells(obj)
           
            zNoCells = size(obj.F,3);
            
        end
        
        function X = get.X(obj)
            
            x = obj.gridOrigin(1):obj.cellSize:obj.gridOrigin(1)+obj.cellSize*(obj.xNoCells-1);
            y = obj.gridOrigin(2):obj.cellSize:obj.gridOrigin(2)+obj.cellSize*(obj.yNoCells-1);
            z = obj.gridOrigin(3):obj.cellSize:obj.gridOrigin(3)+obj.cellSize*(obj.zNoCells-1);
            
            [X, ~, ~] = meshgrid(x, y, z);
            
        end
        
        function Y = get.Y(obj)
            
            x = obj.gridOrigin(1):obj.cellSize:obj.gridOrigin(1)+obj.cellSize*(obj.xNoCells-1);
            y = obj.gridOrigin(2):obj.cellSize:obj.gridOrigin(2)+obj.cellSize*(obj.yNoCells-1);
            z = obj.gridOrigin(3):obj.cellSize:obj.gridOrigin(3)+obj.cellSize*(obj.zNoCells-1);
            
            [~, Y, ~] = meshgrid(x, y, z);
            
        end
        
        function Z = get.Z(obj)
            
            x = obj.gridOrigin(1):obj.cellSize:obj.gridOrigin(1)+obj.cellSize*(obj.xNoCells-1);
            y = obj.gridOrigin(2):obj.cellSize:obj.gridOrigin(2)+obj.cellSize*(obj.yNoCells-1);
            z = obj.gridOrigin(3):obj.cellSize:obj.gridOrigin(3)+obj.cellSize*(obj.zNoCells-1);
            
            [~, ~, Z] = meshgrid(x, y, z);
            
        end


        function xLim = get.xLim(obj)
            
            xLim = [obj.gridOrigin(1) obj.gridOrigin(1)+(obj.xNoCells-1)*obj.cellSize];
            
        end
        
        function yLim = get.yLim(obj)
            
            yLim = [obj.gridOrigin(2) obj.gridOrigin(2)+(obj.yNoCells-1)*obj.cellSize];
            
        end
        
        function zLim = get.zLim(obj)
            
            zLim = [obj.gridOrigin(3) obj.gridOrigin(3)+(obj.zNoCells-1)*obj.cellSize];
            
        end
        
    end
    
    methods (Static)
        
        function f = getValue(f, fx, fy, fz, fxy, fxz, fyz, fxyz, dxn, dyn, dzn)
            
            arguments
                f (8,1,:)
                fx (8,1,:)
                fy (8,1,:)
                fz (8,1,:)
                fxy (8,1,:)
                fxz (8,1,:)
                fyz (8,1,:)
                fxyz (8,1,:)
                dxn (1,1,:)
                dyn (1,1,:)
                dzn (1,1,:)
            end
            
            noPoints = size(f,3);
            
            x = [f
                 fx
                 fy
                 fz
                 fxy
                 fxz
                 fyz
                 fxyz];
            
            % Compute coefficients c
            if isa(x, 'double')
                c = pagemtimes(translationGrid.invA, x);
            elseif isa(x, 'optim.problemdef.OptimizationExpression')
                x = reshape(x, 64, noPoints);
                c = translationGrid.invA*x;
                c = reshape(c, 64, 1, noPoints);
            end
            
%             tic;
            dxn_p2 = dxn.^2;
            dxn_p3 = dxn.^3;
            dyn_p2 = dyn.^2;
            dyn_p3 = dyn.^3;
            dzn_p2 = dzn.^2;
            dzn_p3 = dzn.^3;
            
            f = c( 1,1,:)                               + ... % a000
                c( 2,1,:) .* dxn                        + ... % a100
                c( 3,1,:) .* dxn_p2                     + ... % a200
                c( 4,1,:) .* dxn_p3                     + ... % a300
                c( 5,1,:)           .* dyn              + ... % a010
                c( 6,1,:) .* dxn    .* dyn              + ... % a110
                c( 7,1,:) .* dxn_p2 .* dyn              + ... % a210
                c( 8,1,:) .* dxn_p3 .* dyn              + ... % a310
                c( 9,1,:)           .* dyn_p2           + ... % a020
                c(10,1,:) .* dxn    .* dyn_p2           + ... % a120
                c(11,1,:) .* dxn_p2 .* dyn_p2           + ... % a220
                c(12,1,:) .* dxn_p3 .* dyn_p2           + ... % a320
                c(13,1,:)           .* dyn_p3           + ... % a030
                c(14,1,:) .* dxn    .* dyn_p3           + ... % a130
                c(15,1,:) .* dxn_p2 .* dyn_p3           + ... % a230
                c(16,1,:) .* dxn_p3 .* dyn_p3           + ... % a330
                c(17,1,:)                     .* dzn    + ... % a001
                c(18,1,:) .* dxn              .* dzn    + ... % a101
                c(19,1,:) .* dxn_p2           .* dzn    + ... % a201
                c(20,1,:) .* dxn_p3           .* dzn    + ... % a301
                c(21,1,:)           .* dyn    .* dzn    + ... % a011
                c(22,1,:) .* dxn    .* dyn    .* dzn    + ... % a111
                c(23,1,:) .* dxn_p2 .* dyn    .* dzn    + ... % a211
                c(24,1,:) .* dxn_p3 .* dyn    .* dzn    + ... % a311
                c(25,1,:)           .* dyn_p2 .* dzn    + ... % a021
                c(26,1,:) .* dxn    .* dyn_p2 .* dzn    + ... % a121
                c(27,1,:) .* dxn_p2 .* dyn_p2 .* dzn    + ... % a221
                c(28,1,:) .* dxn_p3 .* dyn_p2 .* dzn    + ... % a321
                c(29,1,:)           .* dyn_p3 .* dzn    + ... % a031
                c(30,1,:) .* dxn    .* dyn_p3 .* dzn    + ... % a131
                c(31,1,:) .* dxn_p2 .* dyn_p3 .* dzn    + ... % a231
                c(32,1,:) .* dxn_p3 .* dyn_p3 .* dzn    + ... % a331
                c(33,1,:)                     .* dzn_p2 + ... % a002
                c(34,1,:) .* dxn              .* dzn_p2 + ... % a102
                c(35,1,:) .* dxn_p2           .* dzn_p2 + ... % a202
                c(36,1,:) .* dxn_p3           .* dzn_p2 + ... % a302
                c(37,1,:)           .* dyn    .* dzn_p2 + ... % a012
                c(38,1,:) .* dxn    .* dyn    .* dzn_p2 + ... % a112
                c(39,1,:) .* dxn_p2 .* dyn    .* dzn_p2 + ... % a212
                c(40,1,:) .* dxn_p3 .* dyn    .* dzn_p2 + ... % a312
                c(41,1,:)           .* dyn_p2 .* dzn_p2 + ... % a022
                c(42,1,:) .* dxn    .* dyn_p2 .* dzn_p2 + ... % a122
                c(43,1,:) .* dxn_p2 .* dyn_p2 .* dzn_p2 + ... % a222
                c(44,1,:) .* dxn_p3 .* dyn_p2 .* dzn_p2 + ... % a322
                c(45,1,:)           .* dyn_p3 .* dzn_p2 + ... % a032
                c(46,1,:) .* dxn    .* dyn_p3 .* dzn_p2 + ... % a132
                c(47,1,:) .* dxn_p2 .* dyn_p3 .* dzn_p2 + ... % a232
                c(48,1,:) .* dxn_p3 .* dyn_p3 .* dzn_p2 + ... % a332
                c(49,1,:)                     .* dzn_p3 + ... % a003
                c(50,1,:) .* dxn              .* dzn_p3 + ... % a103
                c(51,1,:) .* dxn_p2           .* dzn_p3 + ... % a203
                c(52,1,:) .* dxn_p3           .* dzn_p3 + ... % a303
                c(53,1,:)           .* dyn    .* dzn_p3 + ... % a013
                c(54,1,:) .* dxn    .* dyn    .* dzn_p3 + ... % a113
                c(55,1,:) .* dxn_p2 .* dyn    .* dzn_p3 + ... % a213
                c(56,1,:) .* dxn_p3 .* dyn    .* dzn_p3 + ... % a313
                c(57,1,:)           .* dyn_p2 .* dzn_p3 + ... % a023
                c(58,1,:) .* dxn    .* dyn_p2 .* dzn_p3 + ... % a123
                c(59,1,:) .* dxn_p2 .* dyn_p2 .* dzn_p3 + ... % a223
                c(60,1,:) .* dxn_p3 .* dyn_p2 .* dzn_p3 + ... % a323
                c(61,1,:)           .* dyn_p3 .* dzn_p3 + ... % a033
                c(62,1,:) .* dxn    .* dyn_p3 .* dzn_p3 + ... % a133
                c(63,1,:) .* dxn_p2 .* dyn_p3 .* dzn_p3 + ... % a233
                c(64,1,:) .* dxn_p3 .* dyn_p3 .* dzn_p3;      % a333
%             toc;

            if isa(x, 'double')
                f = permute(f, [3 2 1]);
            elseif isa(x, 'optim.problemdef.OptimizationExpression')
                f = reshape(f, noPoints, 1, 1);
            end
            
        end
        
        function a = getValuesOfCellCorners(A, xCellRef, yCellRef, zCellRef)
            
            arguments
                A % matrix
                xCellRef (:,1)
                yCellRef (:,1)
                zCellRef (:,1)
            end
            
            assert(numel(xCellRef) == numel(yCellRef));
            assert(numel(xCellRef) == numel(zCellRef));

            % a is tensor of size 8-by-1-by-n, where n = numel(xCellRef)
            % Order of rows in normalized coordinates:
            % row 1: 0,0,0
            % row 2: 1,0,0
            % row 3: 0,1,0
            % row 4: 1,1,0
            % row 5: 0,0,1
            % row 6: 1,0,1
            % row 7: 0,1,1
            % row 8: 1,1,1
            
            a = A(sub2ind(size(A), ...
                 [xCellRef xCellRef+1 xCellRef xCellRef+1 xCellRef xCellRef+1 xCellRef xCellRef+1], ...
                 [yCellRef yCellRef yCellRef+1 yCellRef+1 yCellRef yCellRef yCellRef+1 yCellRef+1], ...
                 [zCellRef zCellRef zCellRef zCellRef zCellRef+1 zCellRef+1 zCellRef+1 zCellRef+1]));
            
            if isa(a, 'double')
                a = permute(a, [2 3 1]);
            elseif isa(a, 'optim.problemdef.OptimizationExpression')
                a = reshape(a', 8, 1, size(a,1));
            end
            
        end
        
        function invA = invA
            
            load('tricubic_interpolation_invA.mat', 'invA');
            
        end
        
    end
    
end

