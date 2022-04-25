%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [params] = generate_params()
params = struct();

Tf = 60*60*24 * 2; % = 2 days
dt = 60 * 10; % = 10 minutes

% model
params.model = struct(...
    'nx', 6, ...
    'nu', 3, ...
    'Mass', 300, ...
    'GravitationalParameter', 3.986e14, ...
    'ScalingMatrix', [1e-6*eye(3), zeros(3); zeros(3), 1e-3*eye(3)], ...
    'TargetRadius', 7000e3, ...
    'TimeStep', dt, ...
    'HorizonLength', ceil(Tf / dt), ...
    'InitialConditionA', [-15e-3; -400e-3; 24.4e-3; 0; 0.0081; 0], ...
    'InitialConditionB', [-20e-3; 400e-3; 24.4e-3; 0; 0.0108; 0], ...
    'InitialConditionC', [0.02; 0.01; -0.005; 0; 0; 0] ...
);

% constraints
params.constraints = struct(...
    'MaxAbsPositionXZ', 0.1, ...
    'MaxAbsPositionY', 1, ...
    'MaxAbsThrust', 1, ...
    'MaxFinalPosDiff' , 3e-4, ...
    'MaxFinalVelDiff',  1e-3 ...
);

params.exercise = struct( ...
    'QdiagOptA', [91.5; 0.0924; 248;0;0;0] ...
);
    
% YOUR CODE HERE

end
