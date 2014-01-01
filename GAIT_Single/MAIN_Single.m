%---------------------------------------------------%
% Single Stance Test Gait                           %
%---------------------------------------------------%
%MAIN_Single

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

loadFileName = 'oldSoln.mat';  %'' = use default;   'oldSoln.mat'

LOW = 1; UPP = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        START --  USER DEFINED                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Configuration parameters:
LEG_LENGTH = [0.6; 1.0];
DURATION = [0.8; 1.2];
MASS = 8;   %(kg) total robot mass
GRAVITY = 9.81;

%Physical parameters
hip_mass_fraction = 0.85;
auxdata.dynamics.m = 0.5*(1-hip_mass_fraction)*MASS;   %(kg) foot mass
auxdata.dynamics.M = hip_mass_fraction*MASS;    %(kg) hip Mass
auxdata.dynamics.g = GRAVITY;   %(m/s^2) Gravitational acceleration

%Store phase information
auxdata.phase = {'S1'};

%Goal information
auxdata.goal.Step_Length = 0.4;

% % % % %COST FUNCTION:
% % % % auxdata.cost.method = 'Squared'; %{'CoT', 'Squared'}
% % % % auxdata.cost.smoothing.power = 1;
% % % % auxdata.cost.smoothing.distance = 1;
% % % % auxdata.cost.negativeWorkCost = 0.5;
% % % % %  1 = pay full cost for negative work
% % % % %  0 = negative work is free
% % % % % -1 = full regeneration

%enforce friction cone at the contacts
CoeffFriction = 0.9;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

%For animation only:
%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.25;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Trajectory Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% SINGLE STANCE

P.Bnd.Duration = [0.1; 1.0];

P.Bnd.States = zeros(2,8);
P.Bnd.States(:,1) = (pi/2)*[-1;1]; % (rad) Leg One absolute angle
P.Bnd.States(:,2) = (pi/2)*[-1;1]; % (rad) Leg Two absolute angle
P.Bnd.States(:,3) = LEG_LENGTH; % (m) Leg One length
P.Bnd.States(:,4) = LEG_LENGTH; % (m) Leg Two length
P.Bnd.States(:,5) = (pi/0.5)*[-1;1]; % (rad/s) Leg One absolute angular rate
P.Bnd.States(:,6) = (pi/0.5)*[-1;1]; % (rad/s) Leg Two absolute angular rate
P.Bnd.States(:,7) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg One extension rate
P.Bnd.States(:,8) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg Two extensioin rate

P.Bnd.InitialStates = zeros(2,8);
P.Bnd.InitialStates(:,1) = (pi/4)*[1;1]; % (rad) Leg One absolute angle
P.Bnd.InitialStates(:,2) = (pi/4)*[-1;-1]; % (rad) Leg Two absolute angle
P.Bnd.InitialStates(:,3) = LEG_LENGTH; % (m) Leg One length
P.Bnd.InitialStates(:,4) = LEG_LENGTH; % (m) Leg Two length
P.Bnd.InitialStates(:,5) = (pi/0.5)*[-1;1]; % (rad/s) Leg One absolute angular rate
P.Bnd.InitialStates(:,6) = (pi/0.5)*[-1;1]; % (rad/s) Leg Two absolute angular rate
P.Bnd.InitialStates(:,7) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg One extension rate
P.Bnd.InitialStates(:,8) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg Two extensioin rate


P.Bnd.FinalStates = zeros(2,8);
P.Bnd.FinalStates(:,1) = (pi/4)*[-1;-1]; % (rad) Leg One absolute angle
P.Bnd.FinalStates(:,2) = (pi/4)*[1;1]; % (rad) Leg Two absolute angle
P.Bnd.FinalStates(:,3) = LEG_LENGTH; % (m) Leg One length
P.Bnd.FinalStates(:,4) = LEG_LENGTH; % (m) Leg Two length
P.Bnd.FinalStates(:,5) = (pi/0.5)*[-1;1]; % (rad/s) Leg One absolute angular rate
P.Bnd.FinalStates(:,6) = (pi/0.5)*[-1;1]; % (rad/s) Leg Two absolute angular rate
P.Bnd.FinalStates(:,7) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg One extension rate
P.Bnd.FinalStates(:,8) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg Two extensioin rate


P.Bnd.Actuators = zeros(2,4);
Ank_Max = 0.2*LEG_LENGTH(UPP)*MASS*GRAVITY;
Hip_Max = 0.8*LEG_LENGTH(UPP)*MASS*GRAVITY;
P.Bnd.Actuators(:,1) = 4*MASS*GRAVITY*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuators(:,2) = 4*MASS*GRAVITY*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuators(:,3) = Ank_Max*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuators(:,4) = Hip_Max*[-1;1]; % (Nm) Hip torque applied to Leg Two from Leg One

P.Bnd.Path = zeros(2,2);
P.Bnd.Path(:,1) = BndContactAngle; % (rad) contact force angle on stance foot
P.Bnd.Path(:,2) = [0;LEG_LENGTH(UPP)]; %(m) height of swing foot

P.Bnd.Integral = [0; 1e5];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  S1  --  Single Stance                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

iphase = 1;

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd.Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd.Duration(UPP);

bounds.phase(iphase).state.lower = P.Bnd.States(LOW,:);
bounds.phase(iphase).state.upper = P.Bnd.States(UPP,:);
bounds.phase(iphase).control.lower = P.Bnd.Actuators(LOW,:);
bounds.phase(iphase).control.upper = P.Bnd.Actuators(UPP,:);

bounds.phase(iphase).initialstate.lower = P.Bnd.InitialStates(LOW,:);
bounds.phase(iphase).initialstate.upper = P.Bnd.InitialStates(UPP,:);
bounds.phase(iphase).finalstate.lower = P.Bnd.FinalStates(LOW,:);
bounds.phase(iphase).finalstate.upper = P.Bnd.FinalStates(UPP,:);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd.Integral(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.Integral(UPP);

% Bounds for the path constraints:
bounds.phase(iphase).path.lower = P.Bnd.Path(LOW,:);
bounds.phase(iphase).path.upper = P.Bnd.Path(UPP,:);

% Eventgroup bounds
bounds.eventgroup(1).lower = zeros(1,2);
bounds.eventgroup(1).upper = zeros(1,2);

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%


if strcmp(loadFileName,'')   %Use default (bad) guess
    
    iphase=1;
    
    guess.phase(iphase).time = [0; mean(P.Bnd.Duration)];
    
    meanInitialState = 0.5*(P.Bnd.InitialStates(LOW,:) + P.Bnd.InitialStates(UPP,:));
    meanFinalState = 0.5*(P.Bnd.FinalStates(LOW,:) + P.Bnd.FinalStates(UPP,:));
    guess.phase(iphase).state = [meanInitialState ; meanFinalState ];
    
    meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
    guess.phase(iphase).control = [meanControl; meanControl];
    
    meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
    guess.phase(iphase).integral = meanIntegral;
    
else  %Load guess from file
    iphase=1;
    load(loadFileName);
    guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Gait_SingleStance';
setup.functions.continuous = @Continuous_Single;
setup.functions.endpoint = @Endpoint_Single;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt'; %{'snopt', 'ipopt'};
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-3;
setup.mesh.maxiteration = 2;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
%setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-3;

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOPS2 ---------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;


%--------------------------------------------------------------------------%
%------------------------------- Plot Solution ----------------------------%
%--------------------------------------------------------------------------%

plotInfo = getPlotInfo(output);
figNum = 1;
animation(plotInfo,figNum);

figNums = 2:8;
plotSolution(plotInfo,figNums);

if output.result.nlpinfo==0   %Then successful
    %Save the solution if desired:
    outputPrev = output;
    save('oldSoln.mat','outputPrev');
end
