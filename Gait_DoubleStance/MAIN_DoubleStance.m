%---------------------------------------------------%
% Double Stance Test Gait                           %
%---------------------------------------------------%



%%%% HACK %%%% The whole project is under testing until further notice




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
% {'D',             'S1',                'S2',                'F'     }
% {'Double Stance', 'Single Stance One', 'Single Stance Two', 'Flight'}
%

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

loadFileName = 'oldSoln.mat';  %'' = use default;   'oldSoln.mat'

%Physical parameters
auxdata.dynamics.m1 = 0.3;   %(kg) Foot One mass
auxdata.dynamics.m2 = 0.3;   %(kg) Foot Two mass
auxdata.dynamics.M = 0.8;    %(kg) Hip Mass
auxdata.dynamics.g = 9.81;   %(m/s^2) Gravitational acceleration



%%%% HACK %%%% -- Pass constant states as parameters

STANCE_WIDTH = 0.4;

auxdata.state.x1 = STANCE_WIDTH;
auxdata.state.y1 = 0;
auxdata.state.x2 = 0;
auxdata.state.y2 = 0;

auxdata.state.dx1 = 0;
auxdata.state.dy1 = 0;
auxdata.state.dx2 = 0;
auxdata.state.dy2 = 0;

%%%% DONE %%%%

%auxdata.cost.method = 'squared'; %{'CoT', 'Squared'}
auxdata.cost.smoothing.power = 1e-1;
auxdata.cost.smoothing.distance = 1e-2;
auxdata.cost.negativeWorkCost = 0.5;
%  1 = pay full cost for negative work
%  0 = negative work is free
% -1 = full regeneration

auxdata.phase(1) = 'D';  %Used for plotting and animation

%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.5;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Path Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
CoeffFriction = 0.8;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);
LegLength = [0.6;1.0];   %(m) Length of each leg

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

LOW = 1; UPP = 2;

P.Bnd.State.x0 = [0; STANCE_WIDTH];  %(m) Hip Horizontal Position
P.Bnd.State.y0 = [0.2; 1.2];  %(m) Hip Vertical Position

P.Bnd.State.dx0 = [-2;2];  %(m) Hip Horizontal Velocity
P.Bnd.State.dy0 = [-2;2];  %(m) Hip Vertical Velocity

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%NOTE - The limits on {T1; T2} must be set to zero when those feet are not
%in contact with the ground!

Dyn = auxdata.dynamics;
BipedWeight = Dyn.g*(Dyn.m1 + Dyn.m2 + Dyn.M);
GravityLegTorque = LegLength(UPP)*Dyn.g*max(Dyn.m1, Dyn.m2);
FootWidth = 0.08*LegLength(UPP);
AnkleTorque = FootWidth*Dyn.g*Dyn.M;

P.Bnd.Actuator.F1 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = 0.5*GravityLegTorque*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Timing and Cost Function Limits                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Bnd.Duration = [0.2;1.0];

%Maximum work that the actuators can produce during each phase
P.Bnd.Work = [0; 1000]; %(J)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Phase 1  --  D  --  Double Stance                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;
stateBound = [P.Bnd.State.x0, P.Bnd.State.y0, P.Bnd.State.dx0, P.Bnd.State.dy0];
controlBound = convert(P.Bnd.Actuator);

%Start at time = 0
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = P.Bnd.Duration(LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd.Duration(UPP);

bounds.phase(iphase).state.lower = stateBound(LOW,:);
bounds.phase(iphase).state.upper = stateBound(UPP,:);
bounds.phase(iphase).control.lower = controlBound(LOW,:);
bounds.phase(iphase).control.upper = controlBound(UPP,:);

bounds.phase(iphase).initialstate.lower = [STANCE_WIDTH, 0.6, -2, -2];
bounds.phase(iphase).initialstate.upper = [STANCE_WIDTH, 0.7, 2, 2];

bounds.phase(iphase).finalstate.lower = [0, 0.6, -2, -2];
bounds.phase(iphase).finalstate.upper = [0, 0.7, 2, 2];

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.footTwoContactAngle = BndContactAngle;
P.Cst.legOneLength = LegLength;
P.Cst.legTwoLength = LegLength;
path = packConstraints(P.Cst,'D');
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
    
    P.Guess.State.x0 = [STANCE_WIDTH;0];  %(m) Hip Horizontal Position
    P.Guess.State.y0 = 0.6*[1;1];  %(m) Hip Vertical Position
    P.Guess.State.x1 = [0;0];  %(m) Foot One Horizontal Position
    P.Guess.State.y1 = [0;0];  %(m) Foot One Vertical Position
    P.Guess.State.x2 = STANCE_WIDTH*[1;1];  %(m) Foot Two Horizontal Position
    P.Guess.State.y2 = [0;0];  %(m) Foot Two Vertical Position
    
    P.Guess.State.dx0 = 0.4*[1;1];  %(m) Hip Horizontal Velocity
    P.Guess.State.dy0 = [0;0];  %(m) Hip Vertical Velocity
    P.Guess.State.dx1 = [0;0];  %(m) Foot One Horizontal Velocity
    P.Guess.State.dy1 = [0;0];  %(m) Foot One Vertical Velocity
    P.Guess.State.dx2 = [0;0];  %(m) Foot Two Horizontal Velocity
    P.Guess.State.dy2 = [0;0];  %(m) Foot Two Vertical Velocity
    
    guess.phase(iphase).state   = convert(P.Guess.State);
    
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
setup.mesh.tolerance = 1e-5;
setup.mesh.maxiteration = 10;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-5;

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
