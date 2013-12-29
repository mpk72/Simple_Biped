%---------------------------------------------------%
% Optimal Walking Gait                              %
%---------------------------------------------------%

%MAIN_Walking
%
% This script sets up a trajectory optimization problem to find an optimal
% walking gait for the biped model.
%
% Matthew Kelly
% December 26, 2013
% Cornell University
%
% GAIT: 'D' -> 'S1' -> PERIODIC
% {'Double Stance', 'Single Stance One'}
%


%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../shared;

%Load solution from file?    ( '' for none )
loadFileName = 'oldSoln'; %  {'oldSoln.mat', ''}

%Used throughout
LOW = 1; UPP = 2;

%Physical parameters
auxdata.parameters.m1 = 0.3;   %(kg) Foot One mass
auxdata.parameters.m2 = 0.3;   %(kg) Foot Two mass
auxdata.parameters.M = 0.8;    %(kg) Hip Mass
auxdata.parameters.g = 9.81;   %(m/s^2) Gravitational acceleration

%For now, prescribe the step vector
auxdata.parameters.StepVector = [0.4,0];  %[Dx, Dy]; (m)

%Tell the animation and plotting routines about the gait cycle:
auxdata.phase = {'D','S1'};

%COST FUNCTION:
auxdata.parameters.costMethod = 'Squared'; %{'CoT2', 'Squared'}

%enforce friction cone at the contacts
CoeffFriction = 0.8;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

%For animation only:
%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.25;

%Configuration parameters:
LEG_LENGTH = [0.5; 1.2];
VERTICAL_DOMAIN = [0; 1.0];
HORIZONTAL_DOMAIN = norm(auxdata.parameters.StepVector)*1.1*[-1;1];
DURATION_DOUBLE = [0.05; 0.4]; %Duration in double stance
DURATION_SINGLE = [0.2; 1.0];  %Duration in single stance
MAX_SPEED_X = 3;
MAX_SPEED_Y = 1;

%Bounds on the cost function
P.Bnd.IntCost = [0;1e6];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Bounds on the hip position
P.Bnd.State.x0 = HORIZONTAL_DOMAIN;
P.Bnd.State.y0 = VERTICAL_DOMAIN;
P.Bnd.State.dx0 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy0 = MAX_SPEED_Y*[-1;1];

%Bounds on foot one position
P.Bnd.State.x1 = HORIZONTAL_DOMAIN;
P.Bnd.State.y1 = VERTICAL_DOMAIN;
P.Bnd.State.dx1 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy1 = MAX_SPEED_Y*[-1;1];

%Bounds on foot two position
P.Bnd.State.x2 = HORIZONTAL_DOMAIN;
P.Bnd.State.y2 = VERTICAL_DOMAIN;
P.Bnd.State.dx2 = MAX_SPEED_X*[-1;1];
P.Bnd.State.dy2 = MAX_SPEED_Y*[-1;1];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

MAX_FORCE = 200; %(N)
P.Bnd.Actuator.H1 = MAX_FORCE*[-1;1];
P.Bnd.Actuator.V1 = MAX_FORCE*[-1;1];
P.Bnd.Actuator.H2 = MAX_FORCE*[-1;1];
P.Bnd.Actuator.V2 = MAX_FORCE*[-1;1];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  D  --  Double Stance                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1; Phase = 'D';

bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = DURATION_DOUBLE(LOW);
bounds.phase(iphase).finaltime.upper = DURATION_DOUBLE(UPP);

tmpStateBnd = zeros(2,4);
tmpStateBnd(:,1) = P.Bnd.State.x0;
tmpStateBnd(:,2) = P.Bnd.State.y0;
tmpStateBnd(:,3) = P.Bnd.State.dx0;
tmpStateBnd(:,4) = P.Bnd.State.dy0;

bounds.phase(iphase).state.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).state.upper = tmpStateBnd(UPP,:);
bounds.phase(iphase).initialstate.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).initialstate.upper = tmpStateBnd(UPP,:);
bounds.phase(iphase).finalstate.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).finalstate.upper = tmpStateBnd(UPP,:);

tmpCtrlBnd = zeros(2,4);
tmpCtrlBnd(:,1) = P.Bnd.Actuator.H1;
tmpCtrlBnd(:,2) = P.Bnd.Actuator.V1;
tmpCtrlBnd(:,3) = P.Bnd.Actuator.H2;
tmpCtrlBnd(:,4) = P.Bnd.Actuator.V2;

bounds.phase(iphase).control.lower = tmpCtrlBnd(LOW,:);
bounds.phase(iphase).control.upper = tmpCtrlBnd(UPP,:);

% Bounds for the path constraints:
tmpCst = zeros(2,4);
tmpCst(:,1) = LEG_LENGTH.^2;
tmpCst(:,2) = LEG_LENGTH.^2;
tmpCst(:,3) = BndContactAngle;
tmpCst(:,4) = BndContactAngle;
bounds.phase(iphase).path.lower = tmpCst(LOW,:);
bounds.phase(iphase).path.upper = tmpCst(UPP,:);

% Give the bounds for the integral cost function
bounds.phase(iphase).integral.lower = P.Bnd.IntCost(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.IntCost(UPP);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2; Phase = 'S1';

bounds.phase(iphase).initialtime.lower = DURATION_DOUBLE(LOW);
bounds.phase(iphase).initialtime.upper = DURATION_DOUBLE(UPP);
bounds.phase(iphase).finaltime.lower = DURATION_DOUBLE(LOW)+DURATION_SINGLE(LOW);
bounds.phase(iphase).finaltime.upper = DURATION_DOUBLE(UPP)+DURATION_SINGLE(UPP);

tmpStateBnd = zeros(2,8);
tmpStateBnd(:,1) = P.Bnd.State.x0;
tmpStateBnd(:,2) = P.Bnd.State.y0;
tmpStateBnd(:,3) = P.Bnd.State.x2;
tmpStateBnd(:,4) = P.Bnd.State.y2;
tmpStateBnd(:,5) = P.Bnd.State.dx0;
tmpStateBnd(:,6) = P.Bnd.State.dy0;
tmpStateBnd(:,7) = P.Bnd.State.dx2;
tmpStateBnd(:,8) = P.Bnd.State.dy2;

bounds.phase(iphase).state.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).state.upper = tmpStateBnd(UPP,:);
bounds.phase(iphase).initialstate.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).initialstate.upper = tmpStateBnd(UPP,:);
bounds.phase(iphase).finalstate.lower = tmpStateBnd(LOW,:);
bounds.phase(iphase).finalstate.upper = tmpStateBnd(UPP,:);

tmpCtrlBnd = zeros(2,4);
tmpCtrlBnd(:,1) = P.Bnd.Actuator.H1;
tmpCtrlBnd(:,2) = P.Bnd.Actuator.V1;
tmpCtrlBnd(:,3) = P.Bnd.Actuator.H2;
tmpCtrlBnd(:,4) = P.Bnd.Actuator.V2;

bounds.phase(iphase).control.lower = tmpCtrlBnd(LOW,:);
bounds.phase(iphase).control.upper = tmpCtrlBnd(UPP,:);

% Bounds for the path constraints:
tmpCst = zeros(2,3);
tmpCst(:,1) = LEG_LENGTH.^2;
tmpCst(:,2) = LEG_LENGTH.^2;
tmpCst(:,3) = BndContactAngle;
bounds.phase(iphase).path.lower = tmpCst(LOW,:);
bounds.phase(iphase).path.upper = tmpCst(UPP,:);

% Give the bounds for the integral cost function
bounds.phase(iphase).integral.lower = P.Bnd.IntCost(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.IntCost(UPP);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               GUESS                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if strcmp(loadFileName,'')   %Load guess from individual seperate files
    
    iphase = 1;
    guess.phase(iphase).state = [bounds.phase(iphase).state.lower; bounds.phase(iphase).state.upper];
    guess.phase(iphase).control = [bounds.phase(iphase).control.lower; bounds.phase(iphase).control.upper];
    guess.phase(iphase).integral = mean([bounds.phase(iphase).integral.lower; bounds.phase(iphase).integral.upper]);
    guess.phase(iphase).time = [bounds.phase(iphase).initialtime.lower; bounds.phase(iphase).finaltime.upper];
    
    iphase = 2;
    guess.phase(iphase).state = [bounds.phase(iphase).state.lower; bounds.phase(iphase).state.upper];
    guess.phase(iphase).control = [bounds.phase(iphase).control.lower; bounds.phase(iphase).control.upper];
    guess.phase(iphase).integral = mean([bounds.phase(iphase).integral.lower; bounds.phase(iphase).integral.upper]);
    guess.phase(iphase).time = [bounds.phase(iphase).initialtime.lower; bounds.phase(iphase).finaltime.upper];
    
else  %Load guess from file
    load(loadFileName);
    for iphase=1:2;
        guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
        guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
        guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
        guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
    end
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Defect Constraints                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Time Defect 
bounds.eventgroup(1).lower = 0;
bounds.eventgroup(1).upper = 0;

% Hip Defect
bounds.eventgroup(2).lower = zeros(1,4);
bounds.eventgroup(2).upper = zeros(1,4);

% Swing foot defect
bounds.eventgroup(3).lower = zeros(1,4);
bounds.eventgroup(3).upper = zeros(1,4);

% Step vector
bounds.eventgroup(4).lower = zeros(1,2);
bounds.eventgroup(4).upper = zeros(1,2);

% Periodic 
bounds.eventgroup(5).lower = zeros(1,4);
bounds.eventgroup(5).upper = zeros(1,4);

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'GAIT_Walking';
setup.functions.continuous = @Continuous_Walking;
setup.functions.endpoint = @Endpoint_Walking;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-2;
setup.mesh.maxiteration = 3;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 12;
setup.method = 'RPMintegration';
setup.scales.method = 'none'; %{'automatic-bounds','none'};
setup.nlp.options.tolerance = 1e-2;

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
