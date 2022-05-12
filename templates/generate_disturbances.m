%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wt = generate_disturbances(params)
    % Task 25    
    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;
    
    N_t = params.model.HorizonLength;
    nx = params.model.nx;
    
    Wt = zeros(nx,N_t);
    
    for i = 1:nx
        [rows,~] = find(H_w(:,i));
        
        bounds = zeros(length(rows),1);
        for j = rows'
            bounds(j) = h_w(j) / H_w(j,i);
        end
        
        a = min(bounds);
        b = max(bounds);
        
        Wt(i,:) = unifrnd(a,b,[1,N_t]);
    end
end