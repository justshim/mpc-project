%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % Task 1
    % Get parameters
    m = params.model.Mass;
    mu = params.model.GravitationalParameter;
    R = params.model.TargetRadius;
    wn = sqrt(mu/(R^3));
    
    % Define matrices
    Ac = [zeros(3), eye(3);
          3*wn^2, zeros(1,3), 2*wn, 0;
          zeros(1,3), -2*wn, zeros(1,2);
          zeros(1,2), -wn^2, zeros(1,3)];
      
    Bc = [zeros(3); eye(3)/m];
end