%---------------------------------------------------%
% Single Stance One Test Gait                       %
%---------------------------------------------------%

%MAIN_SingleOne
%
% This script set up a simple trajectory optimization that figures out how
% to swing Leg Two from behind the stance Leg One to in front while
% translating the hip horizontally.
%
% Matthew Kelly
% December 22, 2013
% Cornell University
%
% GAIT: 'S1'
% {'D',             'S1',                'S2',                'F'     }
% {'Double Stance', 'Single Stance One', 'Single Stance Two', 'Flight'}
%

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

loadFileName = 'oldSoln.mat';  %'' = use default;   'oldSoln.mat'

LOW = 1; UPP = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        START --  USER DEFINED                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    %Physical parameters
    auxdata.dynamics.m1 = 0.3;   %(kg) Foot One mass
    auxdata.dynamics.m2 = 0.3;   %(kg) Foot Two mass
    auxdata.dynamics.M = 0.8;    %(kg) Hip Mass
    auxdata.dynamics.g = 9.81;   %(m/s^2) Gravitational acceleration

    %Configuration parameters:
    LEG_LENGTH = [0.6; 1.0];
    STEP_LENGTH = [0.3; 0.8];
    HIP_HEIGHT = 0.7;
    DURATION = [0.8; 1.2];
    
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
    CoeffFriction = 0.99;  %Between the foot and the ground
    BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

    %For animation only:
    %1 = Real time, 0.5 = slow motion, 2.0 = fast forward
    auxdata.animation.timeRate = 0.25;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Constants in this Phase                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Constant states
auxdata.constant.x1 = 0;
auxdata.constant.y1 = 0;
auxdata.constant.dx1 = 0;
auxdata.constant.dy1 = 0;

%Constant actuation
auxdata.constant.T2 = 0;

auxdata.phase{1} = 'S1';  %Used for plotting and animation

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    %Bounds on the hip position
    P.Bnd.State.x0 = STEP_LENGTH(UPP)*[-1;1];
    P.Bnd.State.y0 = LEG_LENGTH;
    P.Bnd.State.dx0 = 3*[-1;1];
    P.Bnd.State.dy0 = 1*[-1;1];
    
    config.initialstate.x0 = STEP_LENGTH(LOW)*[-1.1; -0.9];
    config.initialstate.y0 = HIP_HEIGHT*[0.95;1.05];
    config.initialstate.dx0 = 3*[0;1];
    config.initialstate.dy0 = 1*[0;1];
    
    config.finalstate.x0 = STEP_LENGTH(LOW)*[0.9; 1.1];
    config.finalstate.y0 = HIP_HEIGHT*[0.95;1.05];
    config.finalstate.dx0 = 3*[0;1];
    config.finalstate.dy0 = 1*[0;1];
    
    
    %Bounds on the swing foot position
    P.Bnd.State.x2 = STEP_LENGTH(UPP)*[-1;1];
    P.Bnd.State.y2 = [0;0.5];
    P.Bnd.State.dx2 = 3*[-1;1];
    P.Bnd.State.dy2 = 1*[-1;1];
    
    config.initialstate.x2 = STEP_LENGTH(UPP)*[-0.9; -0.7];
    config.initialstate.y2 = [0;0];
    config.initialstate.dx2 = 0.7*[-1;1];
    config.initialstate.dy2 = 0.2*[-1;1];
    
    config.finalstate.x2 = STEP_LENGTH(UPP)*[0.7; 0.9];
    config.finalstate.y2 = [0;0];
    config.finalstate.dx2 = 0.7*[-1;1];
    config.finalstate.dy2 = 0.2*[-1;1];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%NOTE - The limits on {T1; T2} must be set to zero when those feet are not
%in contact with the ground!

Dyn = auxdata.dynamics;
BipedWeight = Dyn.g*(Dyn.m1 + Dyn.m2 + Dyn.M);
GravityLegTorque = LEG_LENGTH(UPP)*Dyn.g*max(Dyn.m1, Dyn.m2);
FootWidth = 0.2*LEG_LENGTH(UPP);   %BIG FEET
AnkleTorque = FootWidth*Dyn.g*Dyn.M;

P.Bnd.Actuator.F1 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.Thip = GravityLegTorque*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Timing and Cost Function Limits                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Bnd.Duration = DURATION;

%Maximum work that the actuators can produce during each phase
if strcmp(auxdata.cost.method,'Squared')
    P.Bnd.Work = [0; 50]; %(Weird Units)
elseif strcmp(auxdata.cost.method,'CoT')
    P.Bnd.Work = [0;25]; %(J)
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  S1  --  Single Stance One                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;
stateBound = convertPartial(P.Bnd.State,'S1');
controlBound = convertPartial(P.Bnd.Actuator,'S1');

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd.Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd.Duration(UPP);

bounds.phase(iphase).state.lower = stateBound(LOW,:);
bounds.phase(iphase).state.upper = stateBound(UPP,:);
bounds.phase(iphase).control.lower = controlBound(LOW,:);
bounds.phase(iphase).control.upper = controlBound(UPP,:);


initState = convertPartial(config.initialstate,'S1');
bounds.phase(iphase).initialstate.lower = initState(LOW,:);
bounds.phase(iphase).initialstate.upper = initState(UPP,:);

finalState = convertPartial(config.finalstate,'S1');
bounds.phase(iphase).finalstate.lower = finalState(LOW,:);
bounds.phase(iphase).finalstate.upper = finalState(UPP,:);

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.legOneLength = LEG_LENGTH;
P.Cst.legTwoLength = LEG_LENGTH;
path = packConstraints(P.Cst,'S1');
bounds.phase(iphase).path.lower = path(LOW,:);
bounds.phase(iphase).path.upper = path(UPP,:);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = P.Bnd.Work(LOW);
bounds.phase(iphase).integral.upper = P.Bnd.Work(UPP);


%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%


if strcmp(loadFileName,'')   %Use default (bad) guess
    
    iphase=1;
    
    guess.phase(iphase).time = [0; mean(P.Bnd.Duration)];
    
    P.Guess.State = [mean(initState,1); mean(finalState,1)];
    
    guess.phase(iphase).state = P.Guess.State;
    
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
setup.functions.continuous = @Continuous_SingleOne;
setup.functions.endpoint = @Endpoint_SingleOne;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'ipopt'; %{'snopt', 'ipopt'};
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-6;
setup.mesh.maxiteration = 5;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
%setup.scales.method = 'automatic-bounds';
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
