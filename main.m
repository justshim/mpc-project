clc
clear
close all

%% MPC Programming Exercise
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Initialization
my_params = generate_params();

%% System Modeling (Tested)
% Task 1
[Ac, Bc] = generate_system_cont(my_params);

% Task 2
[At, Bt] = generate_system(Ac,Bc,my_params);

% Task 3
[A, B] = generate_system_scaled(At,Bt,my_params);

% Task 4
[H_u, h_u, H_x, h_x] = generate_constraints(my_params);

% Task 5 (Done)

%% Unconstrained Optimal Control





