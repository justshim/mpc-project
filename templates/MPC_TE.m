%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TE(Q,R,N,params)
            % Task 18
            nu = params.model.nu;
            nx = params.model.nx;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');

            % Get parameters            
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            
            A = params.model.A;
            B = params.model.B;
            
            % Build constraints and objective over time horizon N
            % x = {x_0} + {x_1, ..., x_N}
            % U = {u_0, ..., u_N-1}
            constraints = [];
            objective = 0;
            x = X0; % Initial state x_0
            
            for i = 1:N
                constraints = [constraints, H_u*U{i} <= h_u, H_x*x <= h_x];
                objective = objective + x'*Q*x +  U{i}'*R*U{i};
                
                % x_i in {x_1, ..., x_N}
                % U{i} in {u_0, ..., u_N-1}
                x = A*x + B*U{i};
            end
            
            % Add terminal equality constraint
            constraints = [constraints, x == zeros(nx,1)];
            
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end