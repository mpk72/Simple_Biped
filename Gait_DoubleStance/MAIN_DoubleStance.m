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
% {'D',             'S1',                'S2',                'F'     }
% {'Double Stance', 'Single Stance One', 'Single Stance Two', 'Flight'}
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

LOW = 1; UPP = 2;

HIP_HEIGHT = 0.7;       %(m) attempt to hold hip at this height
STANCE_WIDTH = 0.3;     %(m) how far apart are the feet
MAX_SPEED = 2.0;    %(m/s) how fast can any point move

P.Bnd.State.x0 = [0; STANCE_WIDTH];   	%(m) Hip horizontal position
P.Bnd.State.y0 = HIP_HEIGHT*[0.7;1.3];      %(m) Hip vertical position
P.Bnd.State.x1 = [0; 0];                %(m) Foot One horizontal position
P.Bnd.State.y1 = [0; 0];                %(m) Foot One vertical position
P.Bnd.State.x2 = STANCE_WIDTH*[1;1];  	%(m) Foot Two horizontal position
P.Bnd.State.y2 = [0; 0];                %(m) Foot Two vertical position

P.Bnd.State.dx0 = MAX_SPEED*[-1;1];	%(m) Hip horizontal velocity
P.Bnd.State.dy0 = [0; 0];           %(m) Hip vertical velocity
P.Bnd.State.dx1 = [0; 0];           %(m) Foot One horizontal velocity
P.Bnd.State.dy1 = [0; 0];           %(m) Foot One vertical velocity
P.Bnd.State.dx2 = [0; 0];           %(m) Foot Two horizontal velocity
P.Bnd.State.dy2 = [0; 0];           %(m) Foot Two vertical velocity


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%NOTE - The limits on {T1, T2} must be set to zero when those feet are not
%in contact with the ground!

%Fow now, just allow huge motors
MAX_TORQUE = 10;  %(Nm) 
MAX_FORCE = 10;   %(N)

P.Bnd.Actuator.F1 = MAX_FORCE*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = MAX_FORCE*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = MAX_TORQUE*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = MAX_TORQUE*[-1;1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = MAX_TORQUE*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Timing and Cost Function Limits                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Bnd.Duration = [0.2,1.0];

%Maximum work that the actuators can produce during each phase
P.Bnd.Work = [0, 1000]; %(J)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Path Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
CoeffFriction = 0.8;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);


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

bounds.phase(iphase).state.lower = stateBound(LOW,:);
bounds.phase(iphase).state.upper = stateBound(UPP,:);
bounds.phase(iphase).control.lower = controlBound(LOW,:);
bounds.phase(iphase).control.upper = controlBound(UPP,:);

% We want the system to be starting with (x,y)->(0,0), (dx,dy)->(0,0)
stateBoundaryConditions = P.Bnd.State;
stateBoundaryConditions.y0 = HIP_HEIGHT*[1;1];
stateBoundaryConditions.dx0 = [0;0];
stateBC = convert(stateBoundaryConditions);
bounds.phase(iphase).initialstate.lower = stateBC(LOW,:);
bounds.phase(iphase).initialstate.upper = stateBC(LOW,:);
bounds.phase(iphase).finalstate.lower = stateBC(UPP,:);
bounds.phase(iphase).finalstate.upper = stateBC(UPP,:);

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.footTwoContactAngle = BndContactAngle;
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
          
    guess.phase(iphase).state   = stateBC;
    
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
