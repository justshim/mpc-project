%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC(Q,R,N,params)
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
 
            Nt = params.model.HorizonLength;
            
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
            
            % TODO: Understand why exactly is this wrong?
            % J_inf(x) = SUM(x'Qx + u'Ru) u = -Kx
            %        ==> SUM(x'Qx + (-x'K')R(-K*x))
            %        ==> SUM(x'(Q+K'RK)x)
%             my_LQR_ctrl = LQR(Q,R,params);
%             K = my_LQR_ctrl.K;
%             P = Q + K'*R*K;

            % Add LQR infinite horizon cost
            % Solution to Discrete Algebraic Riccati Equation
            % J(x_n) = x_n'Px_n
            [P,~,~] = idare(A,B,Q,R,[],[]);
            objective = objective + x'*P*x;
                        
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