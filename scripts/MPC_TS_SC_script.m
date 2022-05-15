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
nu = my_params.model.nu;
nx = my_params.model.nx;

Q = diag(my_params.exercise.QdiagOptA);
R = eye(nu);
N = 30;
[H, h] = lqr_maxPI(Q,R,my_params);

% TODO: Test for multiple different initial conditions within the state
% constraints
% TODO: Test for feasible vs infeasible conditions
% x0 = my_params.model.InitialConditionA; % Feasible
x0 = my_params.model.InitialConditionB; % Feasible
% x0 = my_params.model.InitialConditionC; % Infeasible

%% Simulate MPC w/o Soft Constraints
my_MPC_TS_ctrl = MPC_TS(Q,R,N,H,h,my_params);
[Xt,Ut,u_info] = simulate(x0,my_MPC_TS_ctrl,my_params);

%% Tune S,v for MPC w/ Soft Constraints
S = eye(6);
v = logspace(2,6,5); % 100, 1000, 10000, 100000, 1000000

for i = 1:size(v,2)
    my_MPC_TS_SC_ctrl = MPC_TS_SC(Q,R,N,H,h,S,v(i),my_params);
    [~,Ut_SC,u_info_SC] = simulate(x0,my_MPC_TS_ctrl,my_params);
    
    if Ut_SC == Ut
        save('scripts/MPC_TS_SC_params','S','v');
        break;
    end
end
