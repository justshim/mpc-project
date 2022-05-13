%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% Task 29
    % Get parameters
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    
    X = Polyhedron(H_x, h_x);
    U = Polyhedron(H_u, h_u);
    Eps = Polyhedron(H_tube, h_tube);    
    
    X_tight = minus(X,Eps);
    U_tight = minus(U,K_tube*Eps);
    
    params.constraints.StateMatrix = X_tight.A;
    params.constraints.StateRHS = X_tight.b;
    params.constraints.InputMatrix = U_tight.A;
    params.constraints.InputRHS = U_tight.b;
end