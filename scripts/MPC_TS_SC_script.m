clc
clear
close all

%% MPC Programming Exercise
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Task 23
% Choose S and v such that the controller based on the soft-constrained 
% MPC problem (20) returns the same control input values as the controller 
% based on the MPC problem (19), whenever (19) is feasible, for the same 
% choice of weighting matrices Q∗, R∗ and horizon length N = 30. 
% Verify your selection in simulation by writing a script called 
% MPC_TS_SC_script.m. Provide the script and save your selected values 
% as variables S := S and v := v in the MAT-file MPC_TS_SC_params.mat.

%% Initialization
my_params = generate_params();
N = 30;

% load('lqr_tuning_script', 'q', 'tuning_struct') [TODO: TASK 11]
% Temporary
Q = 0;