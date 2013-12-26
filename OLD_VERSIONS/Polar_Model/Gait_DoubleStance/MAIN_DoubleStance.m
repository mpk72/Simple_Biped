%---------------------------------------------------%
% Double Stance Test Gait                           %
%---------------------------------------------------%

%MAIN_Walking
%
% This script set up a simple trajectory optimization that figures out how
% to move the center of mass of the robot from above one foot to above the
% other foot in a fixed amount of time.
%
% Matthew Kelly
% December 10, 2013
% Cornell University
%
% GAIT: 'D'
% {'Double Stance', 'Single Stance One', 'Single Stance Two'}
%

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

loadFileName = 'oldSoln.mat';  %'' = use default

%Physical parameters
auxdata.dynamics.m1 = 0.3;   %(kg) Foot One mass
auxdata.dynamics.m2 = 0.3;   %(kg) Foot Two mass
auxdata.dynamics.M = 0.8;    %(kg) Hip Mass
auxdata.dynamics.g = 9.81;   %(m/s^2) Gravitational acceleration

auxdata.cost.smoothing.power = 1e-2;
auxdata.cost.smoothing.distance = 1e-3;
auxdata.cost.negativeWorkCost = 0.2;
%  1 = pay full cost for negative work
%  0 = negative work is free
% -1 = full regeneration

auxdata.phase(1) = 'D';  %Used for plotting and animation

%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.5;

P.Cst.HipHeight = 0.6;   %(m) Hip is constrained to move at this height
P.Cst.HipStartPos = 0;   %(m) Hip starts here
P.Cst.HipEndPos = 0.4;   %(m) Hip moves to here

Bounds.eventgroup(1).lower = [P.Cst.HipStartPos, P.Cst.HipHeight];
Bounds.eventgroup(1).upper = [P.Cst.HipStartPos, P.Cst.HipHeight];

Bounds.eventgroup(2).lower = [P.Cst.HipEndPos, P.Cst.HipHeight];
Bounds.eventgroup(2).upper = [P.Cst.HipEndPos, P.Cst.HipHeight];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

LOW = 1; UPP = 2;

LegLength = [0.4, 0.9]; %(m) Bounds on the length of each leg
StepLength = [0.2, 2*LegLength(LOW)]; %(m) Distance between feet in 'D'
StepTime = [0.2, 2.0]; %(s) Duration of half of the complete gait cycle
MaxSwingRate = 5*pi; %(rad/s) Maximum angular rate reachable in swing

P.Bnd.State.x = 3*StepLength(UPP)*[-1, 1];  % (m) Foot One horizontal position
P.Bnd.State.y = [0, 2*LegLength(UPP)];% (m) Foot One vertical position
P.Bnd.State.th1 = pi*[0,1]; % (rad) Leg One absolute angle
P.Bnd.State.th2 = pi*[-1,0]; % (rad) Leg Two absolute angle
P.Bnd.State.L1 = LegLength; % (m) Leg One length
P.Bnd.State.L2 = LegLength; % (m) Leg Two length
P.Bnd.State.dx = MaxSwingRate*LegLength(UPP)*[-1,1]; % (m/s) Foot One horizontal velocity
P.Bnd.State.dy = MaxSwingRate*LegLength(UPP)*[-1,1]; % (m/s) Foot One vertical velocity
P.Bnd.State.dth1 = MaxSwingRate*[-1,1]; % (rad/s) Leg One absolute angular rate
P.Bnd.State.dth2 = MaxSwingRate*[-1,1]; % (rad/s) Leg Two absolute angular rate
P.Bnd.State.dL1 = 10*[-1,1]; % (m/s) Leg One extension rate
P.Bnd.State.dL2 = 10*[-1,1]; % (m/s) Leg Two extensioin rate

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%NOTE - The limits on {T1, T2} must be set to zero when those feet are not
%in contact with the ground!

Dyn = auxdata.dynamics;
BipedWeight = Dyn.g*(Dyn.m1 + Dyn.m2 + Dyn.M);
GravityLegTorque = LegLength(UPP)*Dyn.g*max(Dyn.m1, Dyn.m2);
FootWidth = 0.08*LegLength(UPP);
AnkleTorque = FootWidth*Dyn.g*Dyn.M;

P.Bnd.Actuator.F1 = 2*BipedWeight*[-1,1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 2*BipedWeight*[-1,1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = AnkleTorque*[-1,1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = AnkleTorque*[-1,1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = 0.5*GravityLegTorque*[-1,1]; % (Nm) Torque acting on Leg Two from Leg One


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Timing and Cost Function Limits                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Bnd.Duration = [0.2,1.0];

%Power calculation taken from actuatorPower.m. sum of individual max power
maxRobotPower = ...
    P.Bnd.Actuator.F1(UPP)*P.Bnd.State.dL1(UPP) + ...
    P.Bnd.Actuator.F2(UPP)*P.Bnd.State.dL2(UPP) + ...
    P.Bnd.Actuator.T1(UPP)*P.Bnd.State.dth1(UPP) + ...
    P.Bnd.Actuator.T2(UPP)*P.Bnd.State.dth2(UPP) + ...
    P.Bnd.Actuator.Thip(UPP)*(P.Bnd.State.dth2(UPP)-P.Bnd.State.dth1(UPP));

%Maximum work that the actuators can produce during each phase
P.Bnd.Work = [0, maxRobotPower*P.Bnd.Duration(UPP)]; %(J)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Path Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
CoeffFriction = 0.8;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1,1]; %=atan2(H,V);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  D  --  Double Stance                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;
stateBound = convert(P.Bnd.State);
controlBound = convert(P.Bnd.Actuator);

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd.Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd.Duration(UPP);

bounds.phase(iphase).state.lower = stateBound(:,LOW)';
bounds.phase(iphase).state.upper = stateBound(:,UPP)';
bounds.phase(iphase).control.lower = controlBound(:,LOW)';
bounds.phase(iphase).control.upper = controlBound(:,UPP)';

% We want the system to be starting with (x,y)->(0,0), (dx,dy)->(0,0)
stateTmp = P.Bnd.State;
stateTmp.x = [0,0];
stateTmp.y = [0,0];
stateTmp.dx = [0,0];
stateTmp.dy = [0,0];
stateBnd = convert(stateTmp);
bounds.phase(iphase).initialstate.lower = stateBnd(:,LOW)';
bounds.phase(iphase).initialstate.upper = stateBnd(:,UPP)';

bounds.phase(iphase).finalstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).finalstate.upper = stateBound(:,UPP)';

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.footTwoContactAngle = BndContactAngle;
P.Cst.hipHeight = P.Cst.HipHeight*[1,1];
P.Cst.footOneVel = [0,0];
P.Cst.footTwoVel = [0,0];
path = packConstraints(P.Cst,'D');
bounds.phase(iphase).path.lower = path(:,LOW)';
bounds.phase(iphase).path.upper = path(:,UPP)';

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd.Work(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.Work(UPP);


%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%


if strcmp(loadFileName,'')   %Use default (bad) guess
    
    iphase=1;
    
    guess.phase(iphase).time = [0; mean(P.Bnd.Duration)];
    
    StanceWidth = 0.4;
    StanceHeight = 0.6;
    
    StartState.x = 0;  % (m) Foot One horizontal position
    StartState.y = 0;% (m) Foot One vertical position
    StartState.L1 = 0.7; % (m) Leg One length
    StartState.L2 = sqrt(StanceWidth^2 + StanceHeight^2); % (m) Leg Two length
    StartState.th1 = pi/2; % (rad) Leg One absolute angle
    StartState.th2 = -asin(StartState.L1/StartState.L2); % (rad) Leg Two absolute angle
    StartState.dx = 0; % (m/s) Foot One horizontal velocity
    StartState.dy = 0; % (m/s) Foot One vertical velocity
    StartState.dth1 = 0; % (rad/s) Leg One absolute angular rate
    StartState.dth2 = 0; % (rad/s) Leg Two absolute angular rate
    StartState.dL1 = 0; % (m/s) Leg One extension rate
    StartState.dL2 = 0; % (m/s) Leg Two extensioin rate
    
    FinishState.x = 0;  % (m) Foot One horizontal position
    FinishState.y = 0;% (m) Foot One vertical position
    FinishState.L1 = sqrt(StanceWidth^2 + StanceHeight^2); % (m) Leg One length
    FinishState.L2 = 0.7; % (m) Leg Two length
    FinishState.th1 = asin(FinishState.L2/FinishState.L1); % (rad) Leg One absolute angle
    FinishState.th2 =-pi/2; % (rad) Leg Two absolute angle
    FinishState.dx = 0; % (m/s) Foot One horizontal velocity
    FinishState.dy = 0; % (m/s) Foot One vertical velocity
    FinishState.dth1 = 0; % (rad/s) Leg One absolute angular rate
    FinishState.dth2 = 0; % (rad/s) Leg Two absolute angular rate
    FinishState.dL1 = 0; % (m/s) Leg One extension rate
    FinishState.dL2 = 0; % (m/s) Leg Two extensioin rate
    
    start = convert(StartState);
    finish = convert(FinishState);
    
    guess.phase(iphase).state   = [start';finish'];
    
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
setup.name = 'Gain_Walking';
setup.functions.continuous = @Continuous_DoubleStance;
setup.functions.endpoint = @Endpoint_DoubleStance;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-4;
setup.mesh.maxiteration = 20;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-4;

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

if output.result.nlpinfo==0   %Then successful
    %Save the solution if desired:
    outputPrev = output;
    save('oldSoln.mat','outputPrev');
end
