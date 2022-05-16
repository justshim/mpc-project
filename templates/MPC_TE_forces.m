%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE_forces
    properties
        forces_optimizer
    end

    methods
        function obj = MPC_TE_forces(Q,R,N,params)
            % Task 33
            % Note: Had to use implicit yalmip form to deal with numerical instability
            nu = params.model.nu;
            nx = params.model.nx;
            
            % define optimization variables
            % U = {u_0, ..., u_N-1} = N variables
            % X = {x_0, x_1, ..., x_N+1} = = N+1 variables
            % Slack Eps = {eps_0, eps_1, ..., eps_N+1} = = N+1 variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            
            % Get parameters            
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            
            A = params.model.A;
            B = params.model.B;
             
            % Build constraints and objective over time horizon N
            constraints = [];
            objective = 0;
            
            % Constraints for iterations 0 ~ N-1
            for i = 1:N 
                constraints = [constraints, H_u*U{i} <= h_u, H_x*X{i} <= h_x];
                objective = objective + X{i}'*Q*X{i} +  U{i}'*R*U{i};
                                
                % x_i in {x_1, ..., x_N+1}
                % U{i} in {u_0, ..., u_N-1}
                constraints = [constraints, X{i+1} == A*X{i} + B*U{i}];
            end
            
            % Constraints for iteration = N+1 (x_N)
            % State Variables
            constraints = [constraints, H_x*X{N+1} <= h_x];
            
            % Add LQR infinite horizon cost
            % Solution to Discrete Algebraic Riccati Equation
            % J(x_n) = x_n'Px_n
            [P,~,~] = idare(A,B,Q,R,[],[]);
            objective = objective + X{N+1}'*P*X{N+1};
            
            opts = getOptions('forcesSolver');
            opts.printlevel = 0;
            obj.forces_optimizer = optimizerFORCES(constraints,objective,opts,X{1},{U{1}});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            [optimizer_out,errorcode,info] = obj.forces_optimizer(x);
            u = optimizer_out;
            objective = info.pobj;
            solvetime = info.solvetime;

            feasible = true;
            if any(errorcode ~= 1)
                feasible = false;
                warning('MPC infeasible');
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end