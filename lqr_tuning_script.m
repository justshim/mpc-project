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

%% Parameter Generation
% Sampling broadly in nx space (6D space)
N = 10;

x_rand = logspace(-5,5,N);
[q_1,q_2,q_3,q_4,q_5,q_6] = ndgrid(x_rand,x_rand,x_rand,x_rand,x_rand,x_rand);
Q = [q_1(:),q_2(:),q_3(:),q_4(:),q_5(:),q_6(:)]';

%% Tuning
tic
[tuning_struct, i_opt] = lqr_tuning(x0A,Q,my_params);
toc
q = Q(:,i_opt);

%% Save Results of Parameter Study
% save('lqr_tuning_script', 'q', 'tuning_struct')
