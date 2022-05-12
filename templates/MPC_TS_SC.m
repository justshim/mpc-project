%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC(Q,R,N,H,h,S,v,params)            
            % Task 22
            % Note: Had to use implicit yalmip form to deal with numerical instability
            nu = params.model.nu;
            nx = params.model.nx;
            neps = size(S,1);
            
            % define optimization variables
            % U = {u_0, ..., u_N-1} = N variables
            % X = {x_0, x_1, ..., x_N+1} = = N+1 variables
            % Slack Eps = {eps_0, eps_1, ..., eps_N+1} = = N+1 variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            eps = sdpvar(repmat(neps,1,N+1),ones(1,N+1),'full');
            
            % Get parameters            
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            
            H_eps = -eye(neps);
            h_eps = zeros(neps,1);
            
            A = params.model.A;
            B = params.model.B;
             
            % Build constraints and objective over time horizon N
            constraints = [];
            objective = 0;
            
            % Constraints for iterations 0 ~ N-1
            for i = 1:N 
                constraints = [constraints, H_u*U{i} <= h_u, H_x*X{i} <= h_x + eps{i}];
                objective = objective + X{i}'*Q*X{i} +  U{i}'*R*U{i};
                
                % w/ slack variables
                constraints = [constraints, H_eps * eps{i} <= h_eps];
                objective = objective + eps{i}'*S*eps{i} + v*norm(eps{i},inf);
                
                % x_i in {x_1, ..., x_N+1}
                % U{i} in {u_0, ..., u_N-1}
                constraints = [constraints, X{i+1} == A*X{i} + B*U{i}];
            end
            
            % Constraints for iteration = N+1 (x_N)
            % State Variables
            constraints = [constraints, H_x*X{N+1} <= h_x + eps{N+1}];
            
            % Slack Variables
            constraints = [constraints, H_eps * eps{N+1} <= h_eps];
            objective = objective + eps{i}'*S*eps{i} + v*norm(eps{i},inf);
            
            % Add Maximum Positively Invariant Set Constraint
            constraints = [constraints, H*X{N+1} <= h];
            
            % Add LQR infinite horizon cost
            % Solution to Discrete Algebraic Riccati Equation
            % J(x_n) = x_n'Px_n
            [P,~,~] = idare(A,B,Q,R,[],[]);
            objective = objective + X{N+1}'*P*X{N+1};
            
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X{1},{U{1} objective});
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