clc
clear
close all

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

unconstrainedOptControlTasks = ["simulate", "traj_constraints", ...
                                "lqr_tuning"];
                            
fromLQRtoMPCTasks = ["lqr_maxPI"];
                  
tasksToTest = fromLQRtoMPCTasks;

for i = 1:length(tasksToTest)
    my_test_struct = run_tests(tasksToTest(i));
end

