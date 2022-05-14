clc
clear
close all

init_workspace()

%% MPC Programming Exercise Test Script
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Run All Tests
% my_test_struct = run_tests();

%% Testing
systemModelingTasks = ["generate_system_cont", "generate_system", ...
                       "generate_system_scaled", "generate_constraints", ...
                       "generate_params"];

unconstrainedOptControlTasks = ["simulate", "traj_constraints", "lqr_tuning"];
                            
fromLQRtoMPCTasks = ["lqr_maxPI", "traj_cost", "MPC", "MPC_TE", "MPC_TS"];

softConstraintTasks = ["MPC_TS_SC"];

robustMPCTasks = ["generate_disturbances", "simulate_uncertain", ...
                  "compute_tube_controller", "compute_tightening", ...
                  "MPC_TUBE"]; % , ...
                  % "compute_minRPI"]; % TODO: Numerical issues?

forceProTasks = [];

scripts = ["lqr_tuning_script", "MPC_TS_SC_script", "MPC_Tube_script"];
                  
tasksToTest = robustMPCTasks;

for i = 1:length(tasksToTest)
    my_test_struct = run_tests(tasksToTest(i));
end

