clc
clear
close all

%% MPC Programming Exercise
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Task 31
% Let Q = diag(qz∗,qv∗z), R = 1, N = 50, and choose p = [0.1 0.5] to 
% design the tube controller K_tube using the function designed in Task 27.
% Design the tube E as outlined in Task 28 and obtain tightened constraints
% as outlined in Task 29. Using the function lqr_maxPI with an appropriate 
% choice of inputs, design the terminal set XN such that the resulting 
% controller is recursively feasible and robustly stable. Write the design
% of all ingredients and the setup of the resulting MPC_TUBE controller
% in a script called called MPC_TUBE_script.m. 
% Provide the script and save the following variables in the MAT-file 
% MPC_TUBE_params.mat {p, K_tube, H_tube, h_tube, H_N, h_N, params_z_tube}

%% Initialization 
params = generate_params();
params_z = generate_params_z(params);
Q = params.exercise.QdiagOptA;
Q_z = diag(Q(3:3:6));
R = 1;
N = 50;
p = [0.1,0.5];

%% MPC Tube Controller Design
K_tube = compute_tube_controller(p, params_z);
[H_tube, h_tube,~] = compute_minRPI(K_tube, params_z);
params_z_tube = compute_tightening(K_tube, H_tube, h_tube, params_z);
[H_N, h_N] = lqr_maxPI(Q_z, R, params_z_tube);

%% Save Parameters
save('scripts/MPC_TUBE_params', ...
    'p', 'K_tube', 'H_tube', 'h_tube','H_N', 'h_N', 'params_z_tube');
