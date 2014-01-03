%---------------------------------------------------%
% Double Stance Test Gait                           %
%---------------------------------------------------%
%MAIN_Double

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
DURATION = [0.25; 1.2];
MASS = 8;   %(kg) total robot mass
GRAVITY = 9.81;
STEP_LENGTH = 0.4;

%Physical parameters
hip_mass_fraction = 0.85;
auxdata.dynamics.m = 0.5*(1-hip_mass_fraction)*MASS;   %(kg) foot mass
auxdata.dynamics.M = hip_mass_fraction*MASS;    %(kg) hip Mass
auxdata.dynamics.g = GRAVITY;   %(m/s^2) Gravitational acceleration

%Store phase information
auxdata.phase = {'D'};

%Goal information
auxdata.goal.Step_Length = STEP_LENGTH;
auxdata.dynamics.x2 = auxdata.goal.Step_Length;
auxdata.dynamics.y2 = 0;

%COST FUNCTION:
auxdata.cost.method = 'Squared'; %{'CoT', 'Squared','Work','MOD'}
auxdata.cost.smoothing.power = 0.1;

%enforce friction cone at the contacts
CoeffFriction = 0.9;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

%For animation only:
%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.25;

switch auxdata.cost.method 
    case 'Work'
        Max_Integral = 1e1;
    case 'Squared'
        Max_Integral = 1e-1;
    otherwise
        error('Invalid Cost Function')
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Trajectory Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% SINGLE STANCE

P.Bnd.Duration = DURATION;

P.Bnd.States = zeros(2,4);
P.Bnd.States(:,1) = [0;STEP_LENGTH]; % (m) Hip horizontal position wrt Foot One
P.Bnd.States(:,2) = [0;LEG_LENGTH(UPP)]; % (m) Hip vertical position wrt Foot One
P.Bnd.States(:,3) = 5*[-1;1]; % (m) Hip horizontal velocity
P.Bnd.States(:,4) = 2*[-1;1]; % (m) Hip vertical velocity

P.Bnd.Actuators = zeros(2,2);
Leg_Max = 4*MASS*GRAVITY;

P.Bnd.Actuators(:,1) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuators(:,2) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg Two

P.Bnd.Path = zeros(2,2);
P.Bnd.Path(:,1) = BndContactAngle; % (rad) contact force angle on foot one
P.Bnd.Path(:,2) = BndContactAngle; % (rad) contact force angle on foot two

P.Bnd.Integral = [0; Max_Integral];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  D  --  Double Stance                        %
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

bounds.phase(iphase).initialstate.lower = P.Bnd.States(LOW,:);
bounds.phase(iphase).initialstate.upper = P.Bnd.States(UPP,:);
bounds.phase(iphase).finalstate.lower = P.Bnd.States(LOW,:);
bounds.phase(iphase).finalstate.upper = P.Bnd.States(UPP,:);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd.Integral(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.Integral(UPP);

% Bounds for the path constraints:
bounds.phase(iphase).path.lower = P.Bnd.Path(LOW,:);
bounds.phase(iphase).path.upper = P.Bnd.Path(UPP,:);

% Eventgroup bounds
bounds.eventgroup(1).lower = zeros(1,2); % Hip x Pos
bounds.eventgroup(1).upper = zeros(1,2);
bounds.eventgroup(2).lower = 0.8*ones(1,2)*LEG_LENGTH(UPP); % Hip y Pos
bounds.eventgroup(2).upper = 0.8*ones(1,2)*LEG_LENGTH(UPP);

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%


if strcmp(loadFileName,'')   %Use default (bad) guess
    
    iphase=1;
    
    InitialStates = zeros(1,4);
InitialStates(:,1) = 0.6*STEP_LENGTH; % (m) Hip horizontal position wrt Foot One
InitialStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
InitialStates(:,3) = 0; % (m) Hip horizontal velocity
InitialStates(:,4) = 0; % (m) Hip vertical velocity

FinalStates = zeros(1,4);
FinalStates(:,1) = 0.9*STEP_LENGTH; % (m) Hip horizontal position wrt Foot One
FinalStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
FinalStates(:,3) = 0; % (m) Hip horizontal velocity
FinalStates(:,4) = 0; % (m) Hip vertical velocity
    
    guess.phase(iphase).time = [0; mean(P.Bnd.Duration)];
    
    guess.phase(iphase).state = [InitialStates ; FinalStates ];
    
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Mesh Parameters                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSections = 5;  %Number of sections
nColPts = 8;  %Collocation points per phase

mesh.method = 'hp1';
mesh.tolerance = 1e-6;
mesh.maxiteration = 5;
mesh.phase.colpoints = nColPts*ones(1,nSections);
mesh.phase.fraction  = ones(1,nSections)/nSections;
mesh.colpointsmin = 5;
mesh.colpointsmax = 20;


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Gait_SingleStance';
setup.functions.continuous = @Continuous_Double;
setup.functions.endpoint = @Endpoint_Double;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.nlp.solver = 'snopt'; %{'snopt', 'ipopt'};
setup.derivatives.supplier = 'sparseCD'; %{'sparseBD', 'sparseFD', 'sparseCD'}
setup.derivatives.derivativelevel = 'second'; %{'first','second'};
setup.method = 'RPMintegration';
setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-6;

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;


%--------------------------------------------------------------------------%
%------------------------------- Plot Solution ----------------------------%
%--------------------------------------------------------------------------%

plotInfo = getPlotInfo(output);
figNum = 1;
animation(plotInfo,figNum);

figNums = 2:9;
plotSolution(plotInfo,figNums);

if output.result.nlpinfo==0   %Then successful
    %Save the solution if desired:
    outputPrev = output;
    save('oldSoln.mat','outputPrev');
end
