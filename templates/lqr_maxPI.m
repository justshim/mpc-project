%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix
% OUTPUT:
%   H, h: Describes polytopic X_LQR = {x | H * x <= h}

function [H, h] = lqr_maxPI(Q,R,params)
	% Task 13
    % Get parameters
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;  
    A = params.model.A;
    B = params.model.B;

%     n_x = params.model.nx;
%     n_u = params.model.nu;
    
    % Define LQR Controller
    my_LQR_ctrl = LQR(Q,R,params);
    K = my_LQR_ctrl.K;
    
    % Create initial polytope defined by state constraints
    Phi_i = Polyhedron(H_x,h_x);
    Phi_i.plot
    hold on
    
    % Iterate
    while true
%         % Canonical way of computing Preset of Phi_i
%         % Compute Preset of Phi_i in augmented state + input space
%         F_i = Phi_i.A;
%         f_i = Phi_i.b;
%         
%         F_pre_ = [F_i*A, F_i*B; zeros(size(h_u,1),n_x), H_u];
%         f_pre_ = [f_i;h_u];
%         
%         Pre_Phi_i_ = Polyhedron(F_pre_, f_pre_);
%         
%         % Define Phi_i by projecting back into state space
%         Pre_Phi_i = Pre_Phi_i_.projection(1:n_x);
%         F_pre = Pre_Phi_i.A;
%         f_pre = Pre_Phi_i.b;
        
        % % % % % % % % % % 
        
        % Compute Preset of Phi_i with known LQR input
        % pre(Phi_i) = {x | Ax + Bu in Phi_i}
        % ==> {x | Ax - BKx in Phi_i} since u = -Kx for LQR
        % ==> {x | F(Ax - BKx) <= f} since Phi_i = {x | Fx <= f}
        % ==> {x | F(A-BK)x <= f}
        % Must also satisfy input constraints H_u*u < h_u
        % ==> {u | H_u*u < h_u}
        % ==> {x | (H_u*-K)*x < h_u}
        
        close all
        Phi_i.plot('color','r')
        hold on
        F_i = Phi_i.A;
        f_i = Phi_i.b;
        F_pre = [F_i*(A-B*K); -H_u*K];
        f_pre = [f_i;h_u];
        
        Pre_Phi_i = Polyhedron(F_pre, f_pre);
        Pre_Phi_i.plot('color','g')
                
        % Define Phi_j = intersection of Pre(Phi_i) and Phi_i
        F_j = [F_pre; F_i];
        f_j = [f_pre; f_i];
        Phi_j = Polyhedron(F_j,f_j);
        Phi_j.plot('color','b')
        
        % Check equality Phi_i and Phi_j using subsets
        % Note: Equivalent to checking if Phi_i is contained in Pre(Phi_i)
        if Phi_i == Phi_j
            H = Phi_i.A;
            h = Phi_i.b;
            return
        else
            Phi_i = Phi_j;
        end
    end 
end

