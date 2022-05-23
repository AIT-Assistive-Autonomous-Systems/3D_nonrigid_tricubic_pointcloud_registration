classdef TCIVoxel < handle
    
    methods (Access=public)

        function obj = TCIVoxel

            load("tcivoxel_invA.mat", "invA");
            obj.invA = invA;

        end

        function p = p(obj, xn)

            arguments
                obj
                xn (3,1) double {mustBeNumeric}
            end

            obj.f_to_a;

            p = 0;
            for i = 0:3
                for j = 0:3
                    for k = 0:3
                        p = p + obj.aTensor(i+1,j+1,k+1) * xn(1)^i * xn(2)^j *xn(3)^k;
                    end
                end
            end

        end

    end

    methods

        function set.f(obj, f)

            arguments
                obj
                f (64,1) double {mustBeNumeric}
            end

            obj.f = f;

        end

    end

    methods (Access=private)

        function f_to_a(obj)

            obj.a = obj.invA * obj.f;

            obj.aTensor = NaN(4,4,4);

            obj.aTensor(0+1,0+1,0+1) = obj.a(1); % a000
            obj.aTensor(1+1,0+1,0+1) = obj.a(2); % a100
            obj.aTensor(2+1,0+1,0+1) = obj.a(3); % a200
            obj.aTensor(3+1,0+1,0+1) = obj.a(4); % a300
            obj.aTensor(0+1,1+1,0+1) = obj.a(5); % a010
            obj.aTensor(1+1,1+1,0+1) = obj.a(6); % a110
            obj.aTensor(2+1,1+1,0+1) = obj.a(7); % a210
            obj.aTensor(3+1,1+1,0+1) = obj.a(8); % a310
            obj.aTensor(0+1,2+1,0+1) = obj.a(9); % a020
            obj.aTensor(1+1,2+1,0+1) = obj.a(10); % a120
            obj.aTensor(2+1,2+1,0+1) = obj.a(11); % a220
            obj.aTensor(3+1,2+1,0+1) = obj.a(12); % a320
            obj.aTensor(0+1,3+1,0+1) = obj.a(13); % a030
            obj.aTensor(1+1,3+1,0+1) = obj.a(14); % a130
            obj.aTensor(2+1,3+1,0+1) = obj.a(15); % a230
            obj.aTensor(3+1,3+1,0+1) = obj.a(16); % a330
            obj.aTensor(0+1,0+1,1+1) = obj.a(17); % a001
            obj.aTensor(1+1,0+1,1+1) = obj.a(18); % a101
            obj.aTensor(2+1,0+1,1+1) = obj.a(19); % a201
            obj.aTensor(3+1,0+1,1+1) = obj.a(20); % a301
            obj.aTensor(0+1,1+1,1+1) = obj.a(21); % a011
            obj.aTensor(1+1,1+1,1+1) = obj.a(22); % a111
            obj.aTensor(2+1,1+1,1+1) = obj.a(23); % a211
            obj.aTensor(3+1,1+1,1+1) = obj.a(24); % a311
            obj.aTensor(0+1,2+1,1+1) = obj.a(25); % a021
            obj.aTensor(1+1,2+1,1+1) = obj.a(26); % a121
            obj.aTensor(2+1,2+1,1+1) = obj.a(27); % a221
            obj.aTensor(3+1,2+1,1+1) = obj.a(28); % a321
            obj.aTensor(0+1,3+1,1+1) = obj.a(29); % a031
            obj.aTensor(1+1,3+1,1+1) = obj.a(30); % a131
            obj.aTensor(2+1,3+1,1+1) = obj.a(31); % a231
            obj.aTensor(3+1,3+1,1+1) = obj.a(32); % a331
            obj.aTensor(0+1,0+1,2+1) = obj.a(33); % a002
            obj.aTensor(1+1,0+1,2+1) = obj.a(34); % a102
            obj.aTensor(2+1,0+1,2+1) = obj.a(35); % a202
            obj.aTensor(3+1,0+1,2+1) = obj.a(36); % a302
            obj.aTensor(0+1,1+1,2+1) = obj.a(37); % a012
            obj.aTensor(1+1,1+1,2+1) = obj.a(38); % a112
            obj.aTensor(2+1,1+1,2+1) = obj.a(39); % a212
            obj.aTensor(3+1,1+1,2+1) = obj.a(40); % a312
            obj.aTensor(0+1,2+1,2+1) = obj.a(41); % a022
            obj.aTensor(1+1,2+1,2+1) = obj.a(42); % a122
            obj.aTensor(2+1,2+1,2+1) = obj.a(43); % a222
            obj.aTensor(3+1,2+1,2+1) = obj.a(44); % a322
            obj.aTensor(0+1,3+1,2+1) = obj.a(45); % a032
            obj.aTensor(1+1,3+1,2+1) = obj.a(46); % a132
            obj.aTensor(2+1,3+1,2+1) = obj.a(47); % a232
            obj.aTensor(3+1,3+1,2+1) = obj.a(48); % a332
            obj.aTensor(0+1,0+1,3+1) = obj.a(49); % a003
            obj.aTensor(1+1,0+1,3+1) = obj.a(50); % a103
            obj.aTensor(2+1,0+1,3+1) = obj.a(51); % a203
            obj.aTensor(3+1,0+1,3+1) = obj.a(52); % a303
            obj.aTensor(0+1,1+1,3+1) = obj.a(53); % a013
            obj.aTensor(1+1,1+1,3+1) = obj.a(54); % a113
            obj.aTensor(2+1,1+1,3+1) = obj.a(55); % a213
            obj.aTensor(3+1,1+1,3+1) = obj.a(56); % a313
            obj.aTensor(0+1,2+1,3+1) = obj.a(57); % a023
            obj.aTensor(1+1,2+1,3+1) = obj.a(58); % a123
            obj.aTensor(2+1,2+1,3+1) = obj.a(59); % a223
            obj.aTensor(3+1,2+1,3+1) = obj.a(60); % a323
            obj.aTensor(0+1,3+1,3+1) = obj.a(61); % a033
            obj.aTensor(1+1,3+1,3+1) = obj.a(62); % a133
            obj.aTensor(2+1,3+1,3+1) = obj.a(63); % a233
            obj.aTensor(3+1,3+1,3+1) = obj.a(64); % a333

        end

    end

    properties (Access=public)
        f
    end

    properties (Access=private)
        a
        aTensor
        invA
    end

end