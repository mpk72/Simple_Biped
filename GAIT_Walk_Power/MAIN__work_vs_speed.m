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
INPUT.constraint.duration_single_stance = [0.1; 1.2];
INPUT.constraint.duration_double_stance = [0.02; 0.8];
INPUT.constraint.speed = [0.2; 0.2];
INPUT.constraint.step_distance = 0.4;

%%%% Optimization %%%%
INPUT.optimize.solver = 'snopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tolerance = 1e-4;
INPUT.optimize.max_mesh_iter = 3;

%%%% Cost Function %%%%
INPUT.cost.actuator_weight = 1e-3;
INPUT.cost.actuator_rate_weight = 1e-3;
INPUT.cost.method = 'Work';

%%%% Input / Output parameters %%%%
INPUT.io.loadPrevSoln = true;
INPUT.io.saveSolution = true;
INPUT.io.createPlots = false;
INPUT.io.runAnimation = false;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

N_data_points = 40;
min_speed = 0.15;
max_speed = 2.0;

List_Speeds = linspace(min_speed,max_speed,N_data_points);
List_Work = zeros(size(List_Speeds));
List_exit_code = zeros(size(List_Speeds));

for i=1:N_data_points
    INPUT.constraint.speed = List_Speeds(i)*[1;1];
    if i==1
        INPUT.io.loadPrevSoln = false;
    else
        INPUT.io.loadPrevSoln = true;
    end    
    OUTPUT = Trajectory_Walk(INPUT);
    List_Work(i) = OUTPUT.result.objective;
    List_exit_code(i) = OUTPUT.result.nlpinfo;
end

plot(List_Speeds,List_Work)
title('Work vs Speed')
xlabel('Speed (m/s)')
ylabel('Work (J)')





