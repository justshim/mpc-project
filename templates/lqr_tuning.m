%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    % Task 10
    % Iterate over number of parameters to study
    M = size(Q,2);
    
    % Initialize tuning_struct object
    tuning_struct = [];
    
    % Find lowest cost
    costs = zeros(M,1);
    feas = zeros(M,1);
   
    for i = 1:M
    
        % Define LQR Controller
        my_LQR_ctrl = LQR(diag(Q(:,i)),eye(params.model.nu),params);
        
        % Simulate LQR Controller
        [Xt,Ut,~] = simulate(x0, my_LQR_ctrl, params);
        
        % Evaluate Constraints for Simulated Trajectory and Inputs
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = ...
            traj_constraints(Xt,Ut,params);
        
        costs(i) = J_u;
        feas(i) = traj_feas;
                
        % Build tuning_struct
        tuning_struct_ = struct();
        tuning_struct_.InitialCondition = x0;
        tuning_struct_.Qdiag = Q(:,i);
        tuning_struct_.MaxAbsPositionXZ = s_max;
        tuning_struct_.MaxAbsPositionY = y_max;
        tuning_struct_.MaxAbsThrust = u_max;
        tuning_struct_.InputCost = J_u;
        tuning_struct_.MaxFinalPosDiff = df_max;
        tuning_struct_.MaxFinalVelDiff = vf_max;
        tuning_struct_.TrajFeasible = traj_feas;    
        tuning_struct = [tuning_struct; tuning_struct_];
    end
    
    % Set all infeasible costs to infinity
    costs(~feas) = inf;
    [J_opt, i_opt] = min(costs);
    
    % If best cost is infinity, set index to nan
    if isinf(J_opt)
        i_opt = nan;
    end
        
end