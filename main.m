clc
clear
close all

init_workspace()

%% MPC Programming Exercise Test Script
% Control for Spacecraft Rendezvous
% Justin Shim
% Spring 2022

%% Testing
systemModelingTasks = ["generate_system_cont", "generate_system", ...
                       "generate_system_scaled", "generate_constraints", ...
                       "generate_params"];

unconstrainedOptControlTasks = ["simulate", "traj_constraints", "lqr_tuning"];
                            
fromLQRtoMPCTasks = ["lqr_maxPI", "traj_cost", "MPC", "MPC_TE", "MPC_TS"];

softConstraintTasks = ["MPC_TS_SC"];

% TODO: Numerical issues w/ compute_minRPI?
robustMPCTasks = ["generate_disturbances", "simulate_uncertain", ...
                  "compute_tube_controller", "compute_minRPI", ...
                  "compute_tightening", "MPC_TUBE"]; 
% robustMPCTasks = ["generate_disturbances", "simulate_uncertain", ...
%                   "compute_tube_controller", ...
%                   "compute_tightening", "MPC_TUBE"]; 

forcesProTasks = ["MPC_TE_forces"];

scripts = ["lqr_tuning_script", "MPC_TS_SC_script", "MPC_TUBE_script"];

tasksToTest = scripts;

for i = 1:length(tasksToTest)
    my_test_struct = run_tests(tasksToTest(i));
end

%% Run All Tests
test_struct = run_tests();
save('test_struct', 'test_struct');

%% Final Submission
deliverables = {'generate_system_cont.m', 'generate_system.m', ...
    'generate_system_scaled.m','generate_constraints.m', 'generate_params.m', ...
    'LQR.m', 'simulate.m', 'traj_constraints.m', 'lqr_tuning.m', ...
    'lqr_tuning_script.m', 'lqr_tuning_script.mat', ...
    'lqr_maxPI.m', 'traj_cost.m', 'MPC.m', ...
    'MPC_TE.m', 'MPC_TS.m', ...
    'MPC_TS_SC.m', 'MPC_TS_SC_script.m', 'MPC_TS_SC_params.mat', ...
    'generate_disturbances.m', 'simulate_uncertain.m', ... 
    'compute_tube_controller.m', 'compute_minRPI.m', 'compute_tightening.m', ...
    'MPC_TUBE_script.m', 'MPC_TUBE_params.mat', ...
    'MPC_TE_forces.m', ...
    'test_struct.mat', 'declaration-originality.pdf', 'report.txt'};

zip('MPC22PE_JustinShim.zip', deliverables);
