%---------------------------------------------------%
% Optimal Walking Gait                              %
%---------------------------------------------------%

%MAIN_Walking
%
% This script sets up a trajectory optimization problem to find an optimal
% walking gait for the biped model.
%
% Matthew Kelly
% December 10, 2013
% Cornell University
%
% GAIT: 'D' -> 'S1' -> 'D' -> 'S2' -> PERIODIC
% {'Double Stance', 'Single Stance One', 'Single Stance Two'}
%

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

%Load solution from file?    ( '' for none )
loadFileName = 'oldSoln'; %  {'oldSoln.mat', ''}

%Used throughout
LOW = 1; UPP = 2;

%Physical parameters
auxdata.dynamics.m1 = 0.3;   %(kg) Foot One mass
auxdata.dynamics.m2 = 0.3;   %(kg) Foot Two mass
auxdata.dynamics.M = 0.8;    %(kg) Hip Mass
auxdata.dynamics.g = 9.81;   %(m/s^2) Gravitational acceleration

%Tell the animation and plotting routines about the gait cycle:
auxdata.phase = {'D','S1'};

%COST FUNCTION:
auxdata.cost.method = 'Squared'; %{'CoT', 'Squared'}
auxdata.cost.smoothing.power = 1;
auxdata.cost.smoothing.distance = 1;
auxdata.cost.negativeWorkCost = 0.5;
auxdata.cost.pinionRadius = 0.05;  %(m)  %Convert force to a torque
%  1 = pay full cost for negative work
%  0 = negative work is free
% -1 = full regeneration

%enforce friction cone at the contacts
CoeffFriction = 0.8;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

%For animation only:
%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.25;

%Select the ground slope
auxdata.misc.Ground_Slope = 0;

%Configuration parameters:
LEG_LENGTH = [0.5; 1.0];
HIP_HEIGHT = [0.6; 1.0];
STEP_LENGTH = [0.3; 0.8];
DURATION_DOUBLE = [0.05; 0.4]; %Duration in double stance
DURATION_SINGLE = [0.2; 1.0];  %Duration in single stance
MAX_SPEED_X = 3;
MAX_SPEED_Y = 1;

%Bounds on the cost function
P.Bnd.IntCost = [0;1000];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Decision Parameters                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%Things that are constant, but that the optimization must find.

%For now the only parameter is step length
bounds.parameter.lower = STEP_LENGTH(LOW);
bounds.parameter.upper = STEP_LENGTH(UPP);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Bounds on the hip position
P.Bnd.State.x0 = STEP_LENGTH(UPP)*[-2;2];
P.Bnd.State.y0 = HIP_HEIGHT;
P.Bnd.State.dx0 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy0 = MAX_SPEED_Y*[-1;1];

%Bounds on foot one position
P.Bnd.State.x1 = STEP_LENGTH(UPP)*[-2;2];
P.Bnd.State.y1 = [0; HIP_HEIGHT(UPP)];
P.Bnd.State.dx1 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy1 = MAX_SPEED_Y*[-1;1];

%Bounds on foot two position
P.Bnd.State.x2 = STEP_LENGTH(UPP)*[-2;2];
P.Bnd.State.y2 = [0; HIP_HEIGHT(UPP)];
P.Bnd.State.dx2 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy2 = MAX_SPEED_Y*[-1;1];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Dyn = auxdata.dynamics;
BipedWeight = Dyn.g*(Dyn.m1 + Dyn.m2 + Dyn.M);
GravityLegTorque = LEG_LENGTH(UPP)*Dyn.g*max(Dyn.m1, Dyn.m2);
FootWidth = 0.1*LEG_LENGTH(UPP);   %BIG FEET
AnkleTorque = FootWidth*Dyn.g*Dyn.M;

P.Bnd.Actuator.F1 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = 4*AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = 4*AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = 0.8*GravityLegTorque*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Constraint Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Cst.defect_12 = zeros(1,8);
P.Cst.periodic = zeros(1,7);
P.Cst.time = zeros(1,1);
CstData = packConstraints(P.Cst,'event_walking');
for i=1:length(CstData)
    bounds.eventgroup(i).lower = CstData(i).event;
    bounds.eventgroup(i).upper = CstData(i).event;
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  D  --  Double Stance                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1; Phase = 'D';

stateBound = convertPartial(P.Bnd.State,Phase);
controlBound = convertPartial(P.Bnd.Actuator,Phase);

bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = DURATION_DOUBLE(LOW);
bounds.phase(iphase).finaltime.upper = DURATION_DOUBLE(UPP);

bounds.phase(iphase).state.lower = stateBound(LOW,:);
bounds.phase(iphase).state.upper = stateBound(UPP,:);
bounds.phase(iphase).initialstate.lower = stateBound(LOW,:);
bounds.phase(iphase).initialstate.upper = stateBound(UPP,:);
bounds.phase(iphase).finalstate.lower = stateBound(LOW,:);
bounds.phase(iphase).finalstate.upper = stateBound(UPP,:);

bounds.phase(iphase).control.lower = controlBound(LOW,:);
bounds.phase(iphase).control.upper = controlBound(UPP,:);

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.footTwoContactAngle = BndContactAngle;
P.Cst.legOneLength = LEG_LENGTH;
P.Cst.legTwoLength = LEG_LENGTH;
path = packConstraints(P.Cst,Phase);
bounds.phase(iphase).path.lower = path(LOW,:);
bounds.phase(iphase).path.upper = path(UPP,:);

% Give the bounds for the integral cost function
bounds.phase(iphase).integral.lower = P.Bnd.IntCost(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.IntCost(UPP);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2; Phase = 'S1';

stateBound = convertPartial(P.Bnd.State,Phase);
controlBound = convertPartial(P.Bnd.Actuator,Phase);

bounds.phase(iphase).initialtime.lower = DURATION_DOUBLE(LOW);
bounds.phase(iphase).initialtime.upper = DURATION_DOUBLE(UPP);
bounds.phase(iphase).finaltime.lower = 2*DURATION_DOUBLE(LOW);
bounds.phase(iphase).finaltime.upper = 2*DURATION_DOUBLE(UPP);

bounds.phase(iphase).state.lower = stateBound(LOW,:);
bounds.phase(iphase).state.upper = stateBound(UPP,:);
bounds.phase(iphase).initialstate.lower = stateBound(LOW,:);
bounds.phase(iphase).initialstate.upper = stateBound(UPP,:);
bounds.phase(iphase).finalstate.lower = stateBound(LOW,:);
bounds.phase(iphase).finalstate.upper = stateBound(UPP,:);

bounds.phase(iphase).control.lower = controlBound(LOW,:);
bounds.phase(iphase).control.upper = controlBound(UPP,:);

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.legOneLength = LEG_LENGTH;
P.Cst.legTwoLength = LEG_LENGTH;
path = packConstraints(P.Cst,Phase);
bounds.phase(iphase).path.lower = path(LOW,:);
bounds.phase(iphase).path.upper = path(UPP,:);

% Give the bounds for the integral cost function
bounds.phase(iphase).integral.lower = P.Bnd.IntCost(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.IntCost(UPP);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               GUESS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



if strcmp(loadFileName,'')   %Load guess from individual seperate files
    
    iphase = 1;
    load ../Gait_DoubleStance/Two_to_One.mat;
    guess.phase(iphase).state = outputPrev.result.solution.phase(1).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(1).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(1).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(1).time;
    
    iphase = 2;
    load ../Gait_SingleOne/oldSoln.mat;
    guess.phase(iphase).state = outputPrev.result.solution.phase(1).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(1).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(1).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(1).time;
    
    guess.parameter = mean(STEP_LENGTH);
    
else  %Load guess from file
    load(loadFileName);
    for iphase=1:2;
        guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
        guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
        guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
        guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
    end
    guess.parameter = outputPrev.result.solution.parameter;
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Gain_Walking';
setup.functions.continuous = @Continuous_Walking;
setup.functions.endpoint = @Endpoint_Walking;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-6;
setup.mesh.maxiteration = 2;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 12;
setup.method = 'RPMintegration';
setup.scales.method = 'none'; %{'automatic-bounds','none'};
setup.nlp.options.tolerance = 1e-6;

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
