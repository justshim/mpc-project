%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube; 

            % Task 30
            % Note: Had to use implicit yalmip form to deal with numerical instability
            nu = params.model.nu;
            nx = params.model.nx;
            
            % define optimization variables
            % V = {u_0, ..., u_N-1} = N variables
            % Z = {x_0, x_1, ..., x_N+1} = = N+1 variables
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            Z = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            Xk = sdpvar(nx,1,'full');
            
            % Get parameters       
            % Compute tightened constraints
            params_z = compute_tightening(K_tube,H_tube,h_tube,params); 
            H_u = params_z.constraints.InputMatrix;
            h_u = params_z.constraints.InputRHS;
            H_x = params_z.constraints.StateMatrix;
            h_x = params_z.constraints.StateRHS;
                        
            A = params_z.model.A;
            B = params_z.model.B;
             
            % Build constraints and objective over time horizon N
            constraints = [];
            objective = 0;
            
            % Constraints for initial state (z_0)
            % State Variables
            % exists e in E s.t. z0 + e = xk
            % => e = xk - z0
            % => E = {e : H_tube*e <= h_tube}
            % => E = {e : H_tube*(xk-z0) <= h_tube}
            constraints = [constraints, H_tube*(Xk-Z{1}) <= h_tube];
            
            % Constraints for iterations 0 ~ N-1
            for i = 1:N 
                constraints = [constraints, H_u*V{i} <= h_u, H_x*Z{i} <= h_x];
                objective = objective + Z{i}'*Q*Z{i} +  V{i}'*R*V{i};
                                
                % x_i in {x_1, ..., x_N+1}
                % U{i} in {u_0, ..., u_N-1}
                constraints = [constraints, Z{i+1} == A*Z{i} + B*V{i}];
            end
            
            % Constraints for iteration = N+1 (x_N)
            % State Variables
            constraints = [constraints, H_x*Z{N+1} <= h_x];
                        
            % Add Maximum Positively Invariant Set Constraint
            constraints = [constraints, H_N*Z{N+1} <= h_N];
            
            % Add LQR infinite horizon cost
            % Solution to Discrete Algebraic Riccati Equation
            % J(x_n) = x_n'Px_n
            [P,~,~] = idare(A,B,Q,R,[],[]);
            objective = objective + Z{N+1}'*P*Z{N+1};

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,Xk,{V{1},Z{1}, objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [v, z, objective] = optimizer_out{:};
            
            % Define control input
            u = v + obj.K_tube*(x-z);
                      
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end