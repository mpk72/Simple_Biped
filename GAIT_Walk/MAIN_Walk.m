%---------------------------------------------------%
% Walking Test Gait                                 %
%---------------------------------------------------%
%MAIN_Walk

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

loadPrevSoln = true;
saveSolution = false;

LOW = 1; UPP = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        START --  USER DEFINED                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Configuration parameters:
LEG_LENGTH = [0.6; 1.0];
DURATION_SINGLE = [0.25; 1.2];
DURATION_DOUBLE = [0.1; 0.8];
MASS = 8;   %(kg) total robot mass
GRAVITY = 9.81;
STEP_VECTOR = [0.4;-0.15];  %[horizontal; vertical]

%Common optimization parameters:
TOLERANCE = 1e-5;
MAX_MESH_ITER = 5;

%Actuator Limits
Ank_Max = 0.2*LEG_LENGTH(UPP)*MASS*GRAVITY;
Hip_Max = 0.8*LEG_LENGTH(UPP)*MASS*GRAVITY;
Leg_Max = 2*MASS*GRAVITY;

%Store phase information
auxdata.phase = {'D','S1'};

%Physical parameters
hip_mass_fraction = 0.85;
auxdata.dynamics.m = 0.5*(1-hip_mass_fraction)*MASS;   %(kg) foot mass
auxdata.dynamics.M = hip_mass_fraction*MASS;    %(kg) hip Mass
auxdata.dynamics.g = GRAVITY;   %(m/s^2) Gravitational acceleration

%Step information
auxdata.goal.Step_Vector = STEP_VECTOR;
auxdata.dynamics.x2 = -STEP_VECTOR(1);
auxdata.dynamics.y2 = -STEP_VECTOR(2);

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
        Max_Integrand = 1e1;
    case 'Squared'
        Max_Integrand = 1e-1;
    otherwise
        error('Invalid Cost Function')
end

%Load the previous solution, if available
if loadPrevSoln
    load(['oldSoln_' auxdata.cost.method '.mat']);
end

%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%                                                                   %%%%
%%%%              Phase 1  --  D  --  Double Stance                    %%%%
%%%%                                                                   %%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%

iphase = 1;

P.Bnd(iphase).Duration = DURATION_DOUBLE;

P.Bnd(iphase).States = zeros(2,4);
P.Bnd(iphase).States(:,1) = STEP_VECTOR(1)*[-1;1]; % (m) Hip horizontal position wrt Foot One
P.Bnd(iphase).States(:,2) = STEP_VECTOR(2)*[-1;1] + [0;LEG_LENGTH(2)]; % (m) Hip vertical position wrt Foot One
P.Bnd(iphase).States(:,3) = 5*[-1;1]; % (m) Hip horizontal velocity
P.Bnd(iphase).States(:,4) = 2*[-1;1]; % (m) Hip vertical velocity

P.Bnd(iphase).Actuators = zeros(2,2);
P.Bnd(iphase).Actuators(:,1) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).Actuators(:,2) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg Two

P.Bnd(iphase).Path = zeros(2,2);
P.Bnd(iphase).Path(:,1) = BndContactAngle; % (rad) contact force angle on foot one
P.Bnd(iphase).Path(:,2) = BndContactAngle; % (rad) contact force angle on foot two

P.Bnd(iphase).Integral = [0; Max_Integrand*DURATION_DOUBLE(UPP)];

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd(iphase).Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd(iphase).Duration(UPP);

bounds.phase(iphase).state.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).state.upper = P.Bnd(iphase).States(UPP,:);
bounds.phase(iphase).control.lower = P.Bnd(iphase).Actuators(LOW,:);
bounds.phase(iphase).control.upper = P.Bnd(iphase).Actuators(UPP,:);

bounds.phase(iphase).initialstate.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).initialstate.upper = P.Bnd(iphase).States(UPP,:);
bounds.phase(iphase).finalstate.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).finalstate.upper = P.Bnd(iphase).States(UPP,:);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd(iphase).Integral(LOW);
bounds.phase(iphase).integral.upper = P.Bnd(iphase).Integral(UPP);

% Bounds for the path constraints:
bounds.phase(iphase).path.lower = P.Bnd(iphase).Path(LOW,:);
bounds.phase(iphase).path.upper = P.Bnd(iphase).Path(UPP,:);

%GUESS
if ~loadPrevSoln  %Use default (bad) guess
    
    InitialStates = zeros(1,4);
    InitialStates(:,1) = -0.6*STEP_VECTOR(1); % (m) Hip horizontal position wrt Foot One
    InitialStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
    InitialStates(:,3) = 0; % (m) Hip horizontal velocity
    InitialStates(:,4) = 0; % (m) Hip vertical velocity
    
    FinalStates = zeros(1,4);
    FinalStates(:,1) = -0.1*STEP_VECTOR(1); % (m) Hip horizontal position wrt Foot One
    FinalStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
    FinalStates(:,3) = 0; % (m) Hip horizontal velocity
    FinalStates(:,4) = 0; % (m) Hip vertical velocity
    
    guess.phase(iphase).time = [0; mean(P.Bnd(iphase).Duration)];
    
    guess.phase(iphase).state = [InitialStates ; FinalStates ];
    
    meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
    guess.phase(iphase).control = [meanControl; meanControl];
    
    meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
    guess.phase(iphase).integral = meanIntegral;
    
else  %Load guess from file
    iphase=1;
    
    guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
end


%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%                                                                   %%%%
%%%%              Phase 2  --  S1  --  Single Stance                   %%%%
%%%%                                                                   %%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%
iphase = 2;

P.Bnd(iphase).Duration = DURATION_SINGLE;

P.Bnd(iphase).States = zeros(2,8);
P.Bnd(iphase).States(:,1) = (pi/2)*[-1;1]; % (rad) Leg One absolute angle
P.Bnd(iphase).States(:,2) = (pi/2)*[-1;1]; % (rad) Leg Two absolute angle
P.Bnd(iphase).States(:,3) = LEG_LENGTH; % (m) Leg One length
P.Bnd(iphase).States(:,4) = LEG_LENGTH; % (m) Leg Two length
P.Bnd(iphase).States(:,5) = (pi/0.5)*[-1;1]; % (rad/s) Leg One absolute angular rate
P.Bnd(iphase).States(:,6) = (pi/0.5)*[-1;1]; % (rad/s) Leg Two absolute angular rate
P.Bnd(iphase).States(:,7) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg One extension rate
P.Bnd(iphase).States(:,8) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg Two extensioin rate

P.Bnd(iphase).Actuators = zeros(2,4);
auxdata.cost.Scale.ankle = Ank_Max;
auxdata.cost.Scale.hip = Hip_Max;
auxdata.cost.Scale.leg = Leg_Max;
P.Bnd(iphase).Actuators(:,1) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).Actuators(:,2) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd(iphase).Actuators(:,3) = Ank_Max*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd(iphase).Actuators(:,4) = Hip_Max*[-1;1]; % (Nm) Hip torque applied to Leg Two from Leg One

P.Bnd(iphase).Path = zeros(2,2);
P.Bnd(iphase).Path(:,1) = BndContactAngle; % (rad) contact force angle on stance foot
%%%% HACK %%%%    %These bounds are pretend
P.Bnd(iphase).Path(:,2) = [-abs(STEP_VECTOR(2)); 2*LEG_LENGTH(UPP)]; %(m) height of swing foot
%%%% DONE %%%%
P.Bnd(iphase).Integral = [0; Max_Integrand*DURATION_SINGLE(UPP)];

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd(iphase).Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd(iphase).Duration(UPP);

bounds.phase(iphase).state.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).state.upper = P.Bnd(iphase).States(UPP,:);
bounds.phase(iphase).control.lower = P.Bnd(iphase).Actuators(LOW,:);
bounds.phase(iphase).control.upper = P.Bnd(iphase).Actuators(UPP,:);

bounds.phase(iphase).initialstate.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).initialstate.upper = P.Bnd(iphase).States(UPP,:);
bounds.phase(iphase).finalstate.lower = P.Bnd(iphase).States(LOW,:);
bounds.phase(iphase).finalstate.upper = P.Bnd(iphase).States(UPP,:);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd(iphase).Integral(LOW);
bounds.phase(iphase).integral.upper = P.Bnd(iphase).Integral(UPP);

% Bounds for the path constraints:
bounds.phase(iphase).path.lower = P.Bnd(iphase).Path(LOW,:);
bounds.phase(iphase).path.upper = P.Bnd(iphase).Path(UPP,:);

%GUESS
if ~loadPrevSoln   %Use default (bad) guess
    
    InitialStates = mean(P.Bnd(iphase).States,1);
    InitialStates(1) = pi/3;   %Stance leg angle
    InitialStates(2) = -pi/3;   %Swing leg angle
    
    FinalStates = mean(P.Bnd(iphase).States,1);
    FinalStates(1) = -pi/3;   %Stance leg angle
    FinalStates(2) = pi/3;   %Swing leg angle
    
    guess.phase(iphase).time = [0; mean(P.Bnd(iphase).Duration)];
    
    guess.phase(iphase).state = [InitialStates ; FinalStates];
    
    meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
    guess.phase(iphase).control = [meanControl; meanControl];
    
    meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
    guess.phase(iphase).integral = meanIntegral;
    
else  %Load guess from file
    guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Event Group                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Continuous hip motion
bounds.eventgroup(1).lower = zeros(1,4);
bounds.eventgroup(1).upper = zeros(1,4);

%Hip Translation:
tmp = [STEP_VECTOR(1), STEP_VECTOR(2), 0, 0];
bounds.eventgroup(2).lower = tmp; % Hip states @ toe off
bounds.eventgroup(2).upper = tmp;

% initial swing foot speed
bounds.eventgroup(3).lower = zeros(1,2);
bounds.eventgroup(3).upper = zeros(1,2);

%Swing foot targets:
tmp = [...
    -STEP_VECTOR(1),...     %Foot Two, S, initial, horizontal
    STEP_VECTOR(1),...      %Foot Two, S, final, horizontal
    -STEP_VECTOR(2),...     %Foot Two, S, initial, vertical
    STEP_VECTOR(2)];        %Foot Two, S, final, vertical
bounds.eventgroup(4).lower = tmp;
bounds.eventgroup(4).upper = tmp;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Mesh Parameters                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if loadPrevSoln
    mesh = outputPrev.result.setup.mesh;
else
    nSections = 5;  %Number of sections
    nColPts = 8;  %Collocation points per phase
    mesh.phase(1).colpoints = nColPts*ones(1,nSections);
    mesh.phase(1).fraction  = ones(1,nSections)/nSections;
    mesh.phase(2).colpoints = nColPts*ones(1,nSections);
    mesh.phase(2).fraction  = ones(1,nSections)/nSections;
    mesh.colpointsmin = 5;
    mesh.colpointsmax = 20;
end
mesh.method = 'hp1'; % {'hp','hp1'};
mesh.tolerance = TOLERANCE;
mesh.maxiteration = MAX_MESH_ITER;


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Gait_Walk';
setup.functions.continuous = @Continuous_Walk;
setup.functions.endpoint = @Endpoint_Walk;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.nlp.solver = 'ipopt'; %{'snopt', 'ipopt'};
setup.derivatives.supplier = 'sparseCD'; %{'sparseBD', 'sparseFD', 'sparseCD'}
setup.derivatives.derivativelevel = 'second'; %{'first','second'};
setup.method = 'RPMintegration';
setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = TOLERANCE;

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;


%--------------------------------------------------------------------------%
%------------------------------- Plot Solution ----------------------------%
%--------------------------------------------------------------------------%

plotInfo = getPlotInfo(output);
figNums = 2:9;
plotSolution(plotInfo,figNums);
figNum = 1;
animation(plotInfo,figNum);


if (strcmp(setup.nlp.solver,'ipopt') && output.result.nlpinfo==0) ||...
        (strcmp(setup.nlp.solver,'snopt') && output.result.nlpinfo==1)
    if saveSolution
        %Then successful  --  Save the solution:
        outputPrev = output;
        save(['oldSoln_' auxdata.cost.method '.mat'],'outputPrev');
    end
end



