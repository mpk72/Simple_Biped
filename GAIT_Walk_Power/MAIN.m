%---------------------------------------------------%
% Walking Test Gait                                 %
%---------------------------------------------------%
%MAIN_Walk
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

%%%% Physical parameters %%%%
INPUT.physical.leg_length = [0.6; 1.0];
INPUT.physical.totel_mass = 8;  %(kg) total robot mass
INPUT.physical.hip_mass_fraction = 0.85;
INPUT.physical.gravity = 9.81;
INPUT.physical.coeff_friction = 0.8;

%%%% Constraints %%%%
INPUT.constraint.duration_single_stance = [0.02; 2];
INPUT.constraint.duration_double_stance = [0.02; 2];
INPUT.constraint.speed = [0.2; 0.2];
INPUT.constraint.step_distance = 0.4;

%%%% Optimization %%%%
INPUT.optimize.solver = 'snopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tolerance = 1e-8;
INPUT.optimize.max_mesh_iter = 2;

%%%% Cost Function %%%%
INPUT.cost.actuator_weight = 1e-1;   %1e-3
INPUT.cost.actuator_rate_weight = 1e-1;   %1e-3
INPUT.cost.method = 'Work';

%%%% Input / Output parameters %%%%
INPUT.io.loadPrevSoln = false;
INPUT.io.saveSolution = false;
INPUT.io.createPlots = true;
INPUT.io.runAnimation = false;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[OUTPUT, plotInfo] = Trajectory_Walk(INPUT);
