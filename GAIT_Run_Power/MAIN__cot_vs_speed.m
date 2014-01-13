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
INPUT.physical.actuator_leg_saturate = 2;   % (_)*(Mass*Gravity)
INPUT.physical.actuator_hip_saturate = 0.8; % (_)*(Length*Mass*Gravity)
INPUT.physical.actuator_ank_saturate = 0.2; % (_)*(Length*Mass*Gravity)

%%%% Constraints %%%%
INPUT.constraint.duration_single_stance = [0.02; 2];
INPUT.constraint.duration_double_stance = [0.02; 2];
INPUT.constraint.speed = 0.1*[1;1]; %[0.05; 2.0];
INPUT.constraint.step_distance = [0.05; 1.0];
INPUT.constraint.ground_slope = 0;
INPUT.constraint.ground_curvature = 0;
INPUT.constraint.center_clearance = 0.02;

%%%% Optimization %%%%
INPUT.optimize.solver = 'snopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tol_mesh = 1e-4;
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

N_data_points = 50;
min_speed = 0.1;
max_speed = 2.0;
D.speed = linspace(min_speed,max_speed,N_data_points);

DATA = cell(N_data_points,1);
D.success = false(N_data_points,1);
D.CoT = zeros(N_data_points,1);
D.step_length = zeros(N_data_points,1);

SOLVER = INPUT.optimize.solver;

for i=1:N_data_points
    INPUT.constraint.speed = D.speed(i)*[1;1];
    if i==1
        INPUT.io.loadPrevSoln = false;
    else
        INPUT.io.loadPrevSoln = true;
    end
    output = Trajectory_Walk(INPUT);
    DATA{i} = output;
    
    %%%%Now collect someuseful information:
    D.success(i) = (strcmp(SOLVER,'ipopt') && output.result.nlpinfo==0) ||...
        (strcmp(SOLVER,'snopt') && output.result.nlpinfo==1);
    D.CoT(i) = output.result.objective;
    D.step_length(i) = output.result.solution.parameter;
end

figure(100); clf;
subplot(2,1,1); hold on;
for i=1:N_data_points
    if D.success(i)
        subplot(2,1,1); hold on;
        plot(D.speed,D.CoT,'k.','MarkerSize',20)
        subplot(2,1,2); hold on;
        plot(D.speed,D.step_length,'k.','MarkerSize',20)
    else  %Optimization failed here
        subplot(2,1,1); hold on;
        plot(D.speed,D.CoT,'rx')
        subplot(2,1,2); hold on;
        plot(D.speed,D.step_length,'rx')
    end
end

subplot(2,1,1)
title('CoT vs speed')
xlabel('speed (m/s)')
ylabel('mechanical CoT')

subplot(2,1,2)
title('step length vs speed')
xlabel('speed')
ylabel('step length (m)')




