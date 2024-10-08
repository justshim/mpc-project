clc
clear
close all

%% MPC Programming Exercise
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Task 11
% Use the function lqr_tuning you implemented above to identify a 
% parameter vector q, such that the corresponding LQR controller is 
% feasible and has a low input cost Ju ≤ 11 for the initial condition x0. 
% Provide a script file lqr_tuning_script.m which performs the parameter
% study and finally sets the parameter q := q corresponding to the 
% best controller parametrization. Save the variable q, as well as the 
% tuning_struct of your parameter study in the MAT-file lqr_tuning_script.mat.

% Hint 1: The optimal parameters may span several orders of magnitude. 
% Consider the MATLAB functions logspace and ndgrid to get efficient
% parameter samples for the input matrix Q. You might have to perform 
% several rounds of tuning and refine your sampling grid iteratively 
% to arrive at a satisfactory solution.

% Hint 2: Note that the "in-plane" dynamics of the x,y-coordinates are
% decoupled from the "out-of-plane" dynamics of the z-coordinate. 
% This means you can perform the tuning with respect to the parameters 
% qx , qy , qvx , qvy separately from the tuning parameters for 
% the parameters for qz, qvz 

%% Initialization
my_params = generate_params();
x0A = my_params.model.InitialConditionA;
J_u_threshold = 11;
numRounds = 3;
N = 4;
a = 0.5;
b = 3;

x_log = logspace(-3,3,N);

[q_1,q_2,q_3,q_4,q_5,q_6] = ndgrid(x_log,x_log,x_log,x_log,x_log,x_log);
Q = [q_1(:),q_2(:),q_3(:),q_4(:),q_5(:),q_6(:)]';

%% Tuning
% Sampling broadly in nx space (6D space)
for i = 1:numRounds
    [tuning_struct, i_opt] = lqr_tuning(x0A,Q,my_params);

    if ~isnan(i_opt) && tuning_struct(i_opt).InputCost <= J_u_threshold
        q = Q(:,i_opt);
        tuning_struct(i_opt).InputCost
        
        if i == numRounds
            save('scripts/lqr_tuning_script', 'q', 'tuning_struct')
            break;
        end
    else
        disp("Did not identify feasible paramater vector Q");
        break;
    end

    Q = zeros(6,N);

    for j = 1:6
        Q(j,:) = linspace(a*q(j),b*q(j),N);
    end

    [q_1,q_2,q_3,q_4,q_5,q_6] = ndgrid(Q(1,:),Q(2,:),Q(3,:),Q(4,:),Q(5,:),Q(6,:));
    Q = [q_1(:),q_2(:),q_3(:),q_4(:),q_5(:),q_6(:)]';
end

disp('Tuning complete');
fprintf('Input Cost = %.4f\n',tuning_struct(i_opt).InputCost);
