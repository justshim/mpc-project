%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A,B] = generate_system_scaled(At,Bt,params)
    % Task 3
    % Get parameters
    V = params.model.ScalingMatrix;
    
    % Transform Matrices
    % x~(k+1) = A~ x~(k) + B~ u(k)
    % Substitute x(k) = V x~(k) ==> x~(k) = V^-1 x(k)
    % ==> V^-1 x(k+1) = A~ V^-1 x(k) + B~ u(k)
    % ==> (V V^-1) x(k+1) = (V A~ V^-1) x(k) + (V B~) u(k)
    % ==> x(k+1) = (V A~ V^-1) x(k) + (V B~) u(k)
    
    % Note that this transformation amounts to changing the units
    % of the state from [m;m/s] to [Mm;km/s]
    A = (V * At)/(V);
    B = V * Bt;
end