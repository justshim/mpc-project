%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    % Task 4
    % Input Constraints
    u_max = params.constraints.MaxAbsThrust;
    
    H_u(1:2:6,:) = eye(3);
    H_u(2:2:6,:) = -eye(3);
    h_u = repmat(u_max,6,1);
    
    % State Constraints
    x_max = params.constraints.MaxAbsPositionXZ;
    y_max = params.constraints.MaxAbsPositionY;
    z_max = params.constraints.MaxAbsPositionXZ;
    
    H_x(1:2:6,:) = eye(3);
    H_x(2:2:6,:) = -eye(3);
    h_x = kron([x_max; y_max; z_max],[1;1]);
end