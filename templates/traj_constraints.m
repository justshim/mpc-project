%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % Task 9    
    % s_max: max absolute value of x(k) and z(k)
    x_abs = abs(x(1,:));
    z_abs = abs(x(3,:));
    s_max = max(max([x_abs,z_abs]));
    
    % y_max: max absolute value of y(k)
    y_abs = abs(x(2,:));
    y_max = max(max(y_abs));
    
    % u_max: max absolute value of applied thrust
    u_max = max(max(abs(u)));
    
    % J_u: closed loop inifinite horizon cost
    J_u = sum(u(:).^2);
    
    % df_max: distance from target position at Tf
    df_max = sqrt(sum(x(1:3,end).^2));
    
    % vf_max: absolute difference from the target velocity
    vf_max = sqrt(sum(x(4:6,end).^2));
    
    traj_feas = s_max <= params.constraints.MaxAbsPositionXZ && ...
        y_max <= params.constraints.MaxAbsPositionY && ...
        u_max <= params.constraints.MaxAbsThrust && ...
        df_max <= params.constraints.MaxFinalPosDiff && ...
        vf_max <= params.constraints.MaxFinalVelDiff;
    
end

