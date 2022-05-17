%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % Task 28
    % NOTE: Numerical tolerance issues on M1 Mac MATLAB 2021a
    % Different behavior when running in debug vs run mode?
    % Seems to pass tests when running on a different computer...
    
    % Get parameters
    A = params.model.A;
    B = params.model.B;
    
    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;
    
    A_k = A + (B*K_tube);
    
    % Initialize disturbance polytope
    W = Polyhedron(H_w,h_w);
        
    % Initialize polytope
    Phi_i = W;
    
    n_iter = 0;
    
    for ii = 1:1000  
        n_iter = n_iter + 1;
        
        % Minkowski Sum
        Phi_j = plus(Phi_i, (A_k^n_iter) * W);
                 
        if eq(Phi_i.minHRep(), Phi_j.minHRep())
            H_tube = Phi_j.A;
            h_tube = Phi_j.b;
            break;
        end
        
        Phi_i = Phi_j;
    end
    
end