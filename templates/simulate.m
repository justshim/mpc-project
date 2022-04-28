%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate(x0, ctrl, params)
    % Task 7
    % Get parameters
    A = params.model.A;
    B = params.model.B;
    Nt = params.model.HorizonLength;
    nx = params.model.nx;
    nu = params.model.nu;

    % Initialize output variables
    Xt = [x0, zeros(nx,Nt)];
    Ut = zeros(nu,Nt);
    u_info = [];
    
    % Simulate
    for i = 1:Nt
        [Ut(:,i), u_info_i] = ctrl.eval(Xt(:,i));
        u_info = [u_info u_info_i];
        Xt(:,i+1) = A*Xt(:,i) + B*Ut(:,i);
    end

end