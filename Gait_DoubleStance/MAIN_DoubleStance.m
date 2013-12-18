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
    STANCE_WIDTH = 0.4;
    HIP_HEIGHT = 0.7;
    DURATION = [0.8; 1.2];

    config.footOne.x = 0;
    config.footOne.y = 0;
    config.footTwo.x = STANCE_WIDTH;
    config.footTwo.y = 0;

    config.hipBnd.x0 = [0; STANCE_WIDTH];
    config.hipBnd.y0 = LEG_LENGTH;
    config.hipBnd.dx0 = 3*[-1;1];
    config.hipBnd.dy0 = 1*[-1;1];
    
    config.hipStart.x0 = 0.1*[-1;1];
    config.hipStart.y0 = HIP_HEIGHT*[0.95;1.05];
    config.hipStart.dx0 = 0.1*[-1;1];
    config.hipStart.dy0 = 0.1*[-1;1];
    
    config.hipEnd.x0 = STANCE_WIDTH*[0.95;1.05];
    config.hipEnd.y0 = HIP_HEIGHT*[0.95;1.05];
    config.hipEnd.dx0 = 0.1*[-1;1];
    config.hipEnd.dy0 = 0.1*[-1;1];
    

    auxdata.cost.method = 'Squared'; %{'CoT', 'Squared'}
    auxdata.cost.smoothing.power = 1;
    auxdata.cost.smoothing.distance = 1;
    auxdata.cost.negativeWorkCost = 0.5;
    %  1 = pay full cost for negative work
    %  0 = negative work is free
    % -1 = full regeneration

    %enforce friction cone at the contacts
    CoeffFriction = 0.99;  %Between the foot and the ground
    BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);

    %For animation only:
    %1 = Real time, 0.5 = slow motion, 2.0 = fast forward
    auxdata.animation.timeRate = 0.1;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        END --  USER DEFINED                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

auxdata.state.x1 = config.footOne.x;
auxdata.state.y1 = config.footOne.y;
auxdata.state.x2 = config.footTwo.x;
auxdata.state.y2 = config.footTwo.y;

auxdata.state.dx1 = 0;
auxdata.state.dy1 = 0;
auxdata.state.dx2 = 0;
auxdata.state.dy2 = 0;

auxdata.phase(1) = 'D';  %Used for plotting and animation

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Bnd.State = config.hipBnd;  

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Control Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%NOTE - The limits on {T1; T2} must be set to zero when those feet are not
%in contact with the ground!

Dyn = auxdata.dynamics;
BipedWeight = Dyn.g*(Dyn.m1 + Dyn.m2 + Dyn.M);
GravityLegTorque = LEG_LENGTH(UPP)*Dyn.g*max(Dyn.m1, Dyn.m2);
FootWidth = 0.08*LEG_LENGTH(UPP);
AnkleTorque = FootWidth*Dyn.g*Dyn.M;

P.Bnd.Actuator.F1 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 2*BipedWeight*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = AnkleTorque*[-1;1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = 0.5*GravityLegTorque*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One

%%%% HACK %%%%  See if torque limits are the issue

P.Bnd.Actuator.F1 = 50*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd.Actuator.F2 = 50*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd.Actuator.T1 = 50*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd.Actuator.T2 = 50*[-1;1]; % (Nm) External torque applied to Leg Two
P.Bnd.Actuator.Thip = 50*[-1;1]; % (Nm) Torque acting on Leg Two from Leg One

%%%% DONE %%%%


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
%                Phase 1  --  D  --  Double Stance                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;
stateBound = convertPartial(config.hipBnd,'D');
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


initState = convertPartial(config.hipStart,'D');
bounds.phase(iphase).initialstate.lower = initState(LOW,:);
bounds.phase(iphase).initialstate.upper = initState(UPP,:);

finalState = convertPartial(config.hipEnd,'D');
bounds.phase(iphase).finalstate.lower = finalState(LOW,:);
bounds.phase(iphase).finalstate.upper = finalState(UPP,:);

% Bounds for the path constraints:
P.Cst.footOneContactAngle = BndContactAngle;
P.Cst.footTwoContactAngle = BndContactAngle;
P.Cst.legOneLength = LEG_LENGTH;
P.Cst.legTwoLength = LEG_LENGTH;
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
    
    P.Guess.State = [mean(initState,1); mean(finalState,1)];
    
    guess.phase(iphase).state   = P.Guess.State;
    
    %%%% HACK %%%%
% %     meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
% %     guess.phase(iphase).control = [meanControl; meanControl];
    guess.phase(iphase).control = 0.1*(rand(2,5) - 0.5);

% %     meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
% %     guess.phase(iphase).integral = meanIntegral;
    guess.phase(iphase).integral = 100;

    %%%% DONE %%%%
    
    
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
setup.mesh.tolerance = 1e-8;
setup.mesh.maxiteration = 15;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
%setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-8;

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

figNums = 2:6;
plotSolution(plotInfo,figNums);

if output.result.nlpinfo==0   %Then successful
    %Save the solution if desired:
    outputPrev = output;
    save('oldSoln.mat','outputPrev');
end
