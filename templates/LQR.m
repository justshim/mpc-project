%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K
    end
    
    methods
        % Task 6
        %constructor
        function obj = LQR(Q,R,params)
            A = params.model.A;
            B = params.model.B;
            [obj.K,~,~] = dlqr(A,B,Q,R);
        end
        
        function [u, u_info] = eval(obj,x)
            u = -obj.K*x;
            u_info = struct('ctrl_feas',true);
        end
    end
end