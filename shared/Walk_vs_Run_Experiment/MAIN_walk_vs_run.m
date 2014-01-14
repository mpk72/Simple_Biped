%---------------------------------------------------%
% Walking Test Gait                                 %
%---------------------------------------------------%
%MAIN_Walk
clc; clear; addpath ../../computerGeneratedCode; addpath ..;
addpath ../../GAIT_Walk_Power; addpath ../../GAIT_Run_Power;

%%%% Physical parameters %%%%
INPUT.physical.leg_length = [0.6; 1.0];
INPUT.physical.totel_mass = 8;  %(kg) total robot mass
INPUT.physical.hip_mass_fraction = 0.85;
INPUT.physical.gravity = 9.81;
INPUT.physical.coeff_friction = 0.8;
INPUT.physical.actuator_leg_saturate = 2;   % (_)*(Mass*Gravity)
INPUT.physical.actuator_hip_saturate = 0.8; % (_)*(Length*Mass*Gravity)
INPUT.physical.actuator_ank_saturate = 0.2; % (_)*(Length*Mass*Gravity)

%%%% Constraints %%%%
INPUT.constraint.duration_single_stance = [0.02; 2];
INPUT.constraint.duration_double_stance = [0.02; 2];
INPUT.constraint.duration_flight = [0.02; 2];
INPUT.constraint.speed = 0.1*[1;1]; %[0.05; 2.0];
INPUT.constraint.step_distance = [0.05; 1.0];
INPUT.constraint.ground_slope = 0;
INPUT.constraint.ground_curvature = 0;
INPUT.constraint.center_clearance = 0.02;

%%%% Optimization %%%%
INPUT.optimize.solver = 'snopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tol_mesh = 1e-3;
INPUT.optimize.tol_opt = 1e-6;
INPUT.optimize.max_mesh_iter = 2;

%%%% Cost Function %%%%
INPUT.cost.actuator_weight = 1e-2;   %1e-3
INPUT.cost.actuator_rate_weight = 1e-2;   %1e-3
INPUT.cost.method = 'CoT';  %{'Work','CoT'}

%%%% Input / Output parameters %%%%
INPUT.io.loadPrevSoln = true;
INPUT.io.saveSolution = true;
INPUT.io.createPlots = false;
INPUT.io.runAnimation = false;

SOLVER = INPUT.optimize.solver;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

N_data_points = 25;
min_speed = 0.05;
max_speed = 3;
D.speed = linspace(min_speed,max_speed,N_data_points);

WALK = 1; RUN = 2;
Data = cell(N_data_points,2);
D.success = false(N_data_points,2);
D.CoT = zeros(N_data_points,2);
D.step_length = zeros(N_data_points,1);

walk_load_safe = false;
run_load_safe = false;
for i=1:N_data_points
    INPUT.constraint.speed = D.speed(i)*[1;1];
    %%%% WALK %%%%
    INPUT.io.loadPrevSoln = walk_load_safe;
    output = Trajectory_Walk(INPUT);
    Data{i,WALK} = output;
    D.success(i,WALK) = (strcmp(SOLVER,'ipopt') && output.result.nlpinfo==0) ||...
        (strcmp(SOLVER,'snopt') && output.result.nlpinfo==1);
    D.CoT(i,WALK) = output.result.objective;
    D.step_length(i,WALK) = output.result.solution.parameter;
    walk_load_safe = walk_load_safe || D.success(i,WALK);
    %%%% RUN %%%%
    INPUT.io.loadPrevSoln = run_load_safe;
    output = Trajectory_Run(INPUT);
    Data{i,RUN} = output;
    D.success(i,RUN) = (strcmp(SOLVER,'ipopt') && output.result.nlpinfo==0) ||...
        (strcmp(SOLVER,'snopt') && output.result.nlpinfo==1);
    D.CoT(i,RUN) = output.result.objective;
    D.step_length(i,RUN) = output.result.solution.parameter;
    run_load_safe = run_load_safe || D.success(i,RUN);
end

figure(100); clf;
subplot(2,1,1); hold on;
for i=1:N_data_points
    if D.success(i,WALK)
        subplot(2,1,1); hold on;
        plot(D.speed(i),D.CoT(i,WALK),'k.','MarkerSize',20)
        subplot(2,1,2); hold on;
        plot(D.speed(i),D.step_length(i,WALK),'k.','MarkerSize',20)
    else  %Optimization failed here
        subplot(2,1,1); hold on;
        plot(D.speed(i),D.CoT(i,WALK),'rx','MarkerSize',8,'LineWidth',2)
        subplot(2,1,2); hold on;
        plot(D.speed(i),D.step_length(i,WALK),'rx','MarkerSize',8,'LineWidth',2)
    end
    if D.success(i,RUN)
        subplot(2,1,1); hold on;
        plot(D.speed(i),D.CoT(i,RUN),'b*','MarkerSize',8,'LineWidth',2)
        subplot(2,1,2); hold on;
        plot(D.speed(i),D.step_length(i,RUN),'b*','MarkerSize',8,'LineWidth',2)
    else  %Optimization failed here
        subplot(2,1,1); hold on;      
        plot(D.speed(i),D.CoT(i,RUN),'r+','MarkerSize',8,'LineWidth',2)
        subplot(2,1,2); hold on;     
        plot(D.speed(i),D.step_length(i,RUN),'r+','MarkerSize',8,'LineWidth',2)
    end
end

subplot(2,1,1)
plot(D.speed,D.CoT(:,WALK),'k')
plot(D.speed,D.CoT(:,RUN),'b')
title('CoT vs speed')
xlabel('speed (m/s)')
ylabel('mechanical CoT')
legend('Walk', 'Run')

subplot(2,1,2)
plot(D.speed,D.step_length(:,WALK),'k')
plot(D.speed,D.step_length(:,RUN),'b')
title('step length vs speed')
xlabel('speed (m/s)')
ylabel('step length (m)')
legend('Walk', 'Run')



