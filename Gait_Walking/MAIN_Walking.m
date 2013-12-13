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
%                 NOTES                             %
%---------------------------------------------------%

% This does not converge as is. I think that there are two parts two the
% problem. The first is that I'm giving it a HUGE trajectory optimization
% problem. I would probably be better off assuming that the system is
% symetric and doing a right-left map. Later on I can have the controller
% be asymetric if I want.
%
% If that doesn't fix things, then I will need to get a better initial
% guess. One hack would be to just pick a few more points in phase space
% that are reasonably intelligent, based on the double pendulum walking
% model.
%
% A slightly fancier thing to do would be to take the actual solution to
% the double pendulum walker and plug that in directly as the
% initialization.
%

%---------------------------------------------------%
% Misc Setup                                        %
%---------------------------------------------------%
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

%Load solution from file?    ( '' for none )
guessFile = ''; %  'oldSoln.mat'  OR   ''

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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          State Limits                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

LOW = 1; UPP = 2;

LegLength = [0.6, 0.9]; %(m) Bounds on the length of each leg
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
%                  Constrain the speed of the gait                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% HACK %%%%
% Probably should get a better method for indexing eventgroups
bounds.eventgroup(6).lower = 0.2;  %(m/s) slowest allowable speed
bounds.eventgroup(6).upper = 0.4;  %(m/s) fastest allowable speed

bounds.eventgroup(7).lower = 0.4; %(m) shortest allowable 2*(step length)
bounds.eventgroup(7).upper = 1.3; %(m) largest allowable 2*(step length)
%%%% DONE %%%%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Timing and Cost Function Limits                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Timing parameters:
minPhaseDuration = 0.005;   %(s) minimum time allowed per phase
maxPhaseDuration_D = 0.4;   %(s) maximum time allowed in double stance
maxPhaseDuration_S1 = 0.8;  %(s) maximum time allowed in single stance one
maxPhaseDuration_S2 = 0.8;  %(s) maximum time allowed in double stance two
PhaseDurationGuess = [0.1, 0.5, 0.1, 0.5];  %(s) used for default guess

P.Bnd.Time = zeros(4,2);
P.Bnd.Time(1,:) = [minPhaseDuration, maxPhaseDuration_D];
P.Bnd.Time(2,:) = [minPhaseDuration, maxPhaseDuration_S1];
P.Bnd.Time(3,:) = [minPhaseDuration, maxPhaseDuration_D];
P.Bnd.Time(4,:) = [minPhaseDuration, maxPhaseDuration_S2];

nTimeDefects = 4-1;   %Need to match time between phases
bounds.eventgroup(5).lower = zeros(1,nTimeDefects);
bounds.eventgroup(5).upper = zeros(1,nTimeDefects);

%Power calculation taken from actuatorPower.m. sum of individual max power
maxRobotPower = ...
    P.Bnd.Actuator.F1(UPP)*P.Bnd.State.dL1(UPP) + ...
    P.Bnd.Actuator.F2(UPP)*P.Bnd.State.dL2(UPP) + ...
    P.Bnd.Actuator.T1(UPP)*P.Bnd.State.dth1(UPP) + ...
    P.Bnd.Actuator.T2(UPP)*P.Bnd.State.dth2(UPP) + ...
    P.Bnd.Actuator.Thip(UPP)*(P.Bnd.State.dth2(UPP)-P.Bnd.State.dth1(UPP));

%Maximum work that the actuators can produce during each phase
P.Bnd.MaxWork.D = maxRobotPower*maxPhaseDuration_D; %(J) 
P.Bnd.MaxWork.S1 = maxRobotPower*maxPhaseDuration_S1; %(J) 
P.Bnd.MaxWork.S2 = maxRobotPower*maxPhaseDuration_S2; %(J) 


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
bounds.phase(iphase).finaltime.lower = P.Bnd.Time(1,LOW);
bounds.phase(iphase).finaltime.upper = P.Bnd.Time(1,UPP);

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
P.Cst.D.footOneContactAngle = BndContactAngle;
P.Cst.D.footTwoContactAngle = BndContactAngle;
path = packConstraints(P.Cst.D,'D');
bounds.phase(iphase).path.lower = path(:,LOW)';
bounds.phase(iphase).path.upper = path(:,UPP)';

% Ensure that phase map defects are zero
nState = 12;
ievent = iphase;
bounds.eventgroup(ievent).lower = zeros(1,nState);
bounds.eventgroup(ievent).upper = zeros(1,nState);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = P.Bnd.MaxWork.D;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;
stateBound = convert(P.Bnd.State);
ctrlBnd = P.Bnd.Actuator;
ctrlBnd.T2 = [0,0];   %No ankle torque from swing foot
controlBound = convert(ctrlBnd);

bounds.phase(iphase).initialtime.lower = P.Bnd.Time(1,LOW);
bounds.phase(iphase).initialtime.upper = P.Bnd.Time(1,UPP);
bounds.phase(iphase).finaltime.lower = sum(P.Bnd.Time(1:2,LOW));
bounds.phase(iphase).finaltime.upper = sum(P.Bnd.Time(1:2,UPP));

bounds.phase(iphase).initialstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).initialstate.upper = stateBound(:,UPP)';
bounds.phase(iphase).finalstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).finalstate.upper = stateBound(:,UPP)';

bounds.phase(iphase).state.lower = stateBound(:,LOW)'; 
bounds.phase(iphase).state.upper = stateBound(:,UPP)'; 
bounds.phase(iphase).control.lower = controlBound(:,LOW)'; 
bounds.phase(iphase).control.upper = controlBound(:,UPP)';

% Bounds for the path constraints:
P.Cst.S1.footOneContactAngle = BndContactAngle;
P.Cst.S1.footTwoHeight = [0,2*LegLength(UPP)];
path = packConstraints(P.Cst.S1,'S1');
bounds.phase(iphase).path.lower = path(:,LOW)';
bounds.phase(iphase).path.upper = path(:,UPP)';

% Ensure that phase map defects are zero
nState = 12;
ievent = iphase;
bounds.eventgroup(ievent).lower = zeros(1,nState);
bounds.eventgroup(ievent).upper = zeros(1,nState);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = P.Bnd.MaxWork.S1;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 3  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

iphase = 3;
stateBound = convert(P.Bnd.State);
controlBound = convert(P.Bnd.Actuator);

bounds.phase(iphase).initialtime.lower = sum(P.Bnd.Time(1:2,LOW));
bounds.phase(iphase).initialtime.upper = sum(P.Bnd.Time(1:2,UPP));
bounds.phase(iphase).finaltime.lower = sum(P.Bnd.Time(1:3,LOW));
bounds.phase(iphase).finaltime.upper = sum(P.Bnd.Time(1:3,UPP));

bounds.phase(iphase).initialstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).initialstate.upper = stateBound(:,UPP)';
bounds.phase(iphase).finalstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).finalstate.upper = stateBound(:,UPP)';

bounds.phase(iphase).state.lower = stateBound(:,LOW)'; 
bounds.phase(iphase).state.upper = stateBound(:,UPP)'; 
bounds.phase(iphase).control.lower = controlBound(:,LOW)'; 
bounds.phase(iphase).control.upper = controlBound(:,UPP)';

% Bounds for the path constraints:
path = packConstraints(P.Cst.D,'D');
bounds.phase(iphase).path.lower = path(:,LOW)';
bounds.phase(iphase).path.upper = path(:,UPP)';

% Ensure that phase map defects are zero
nState = 12;
ievent = iphase;
bounds.eventgroup(ievent).lower = zeros(1,nState);
bounds.eventgroup(ievent).upper = zeros(1,nState);

% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = P.Bnd.MaxWork.D;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 4  --  S2  --  Single Stance Two                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 4;
stateBound = convert(P.Bnd.State);
ctrlBnd = P.Bnd.Actuator;
ctrlBnd.T1 = [0,0];   %No ankle torque from swing foot
controlBound = convert(ctrlBnd);

bounds.phase(iphase).initialstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).initialstate.upper = stateBound(:,UPP)';
bounds.phase(iphase).finalstate.lower = stateBound(:,LOW)';
bounds.phase(iphase).finalstate.upper = stateBound(:,UPP)';

bounds.phase(iphase).initialtime.lower = sum(P.Bnd.Time(1:3,LOW));
bounds.phase(iphase).initialtime.upper = sum(P.Bnd.Time(1:3,UPP));
bounds.phase(iphase).finaltime.lower = sum(P.Bnd.Time(1:4,LOW));
bounds.phase(iphase).finaltime.upper = sum(P.Bnd.Time(1:4,UPP));

bounds.phase(iphase).state.lower = stateBound(:,LOW)'; 
bounds.phase(iphase).state.upper = stateBound(:,UPP)'; 
bounds.phase(iphase).control.lower = controlBound(:,LOW)'; 
bounds.phase(iphase).control.upper = controlBound(:,UPP)';

% Bounds for the path constraints:
P.Cst.S2.footOneHeight = [0,2*LegLength(UPP)];
P.Cst.S2.footTwoContactAngle = BndContactAngle;
path = packConstraints(P.Cst.S2,'S2');
bounds.phase(iphase).path.lower = path(:,LOW)';
bounds.phase(iphase).path.upper = path(:,UPP)';

% Ensure that phase map defects are zero
% Note that we don't constraint horizontal translation, so there is one
% fewer defect in this phase.
nState = 12;
ievent = iphase;
bounds.eventgroup(ievent).lower = zeros(1,(nState-1));
bounds.eventgroup(ievent).upper = zeros(1,(nState-1));


% Give the bounds for the integral (total actuator work) calculation:
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = P.Bnd.MaxWork.S2;


%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

if strcmp(guessFile, '')  %Then use default - constant at mean(Bnd)
    startTime = 0;
    for iphase=1:4
        endTime = startTime + PhaseDurationGuess(iphase);
        guess.phase(iphase).time    = [startTime; endTime]; 
        meanState = 0.5*(bounds.phase(iphase).state.lower + bounds.phase(iphase).state.upper);
        guess.phase(iphase).state   = [meanState; meanState];
        meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
        guess.phase(iphase).control = [meanControl; meanControl];
        meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
        guess.phase(iphase).integral = meanIntegral;
        startTime = endTime;
        
        %%%% HACK %%%%
            %Inject random noise into guess:
            guess.phase(iphase).control = 0.5*rand(size(guess.phase(iphase).control));
            guess.phase(iphase).state = guess.phase(iphase).state + 0.5*rand(size(guess.phase(iphase).state));
        %%%% DONE %%%%
    end
else %Load from a data file:
    
    error('This option is not yet enabled...')
    
% %     load(guessFile);
% %     guess.phase(iphase).time = outputPrev.result.solution.phase(1).time;
% %     guess.phase(iphase).state = outputPrev.result.solution.phase(1).state;
% %     guess.phase(iphase).control = outputPrev.result.solution.phase(1).control;
% %     guess.phase(iphase).integral = outputPrev.result.objective;
% %     
% %     %Use a mesh that matches with input:
% %     Npts = length(guess.phase(iphase).time);
% %     nColPts = 4;
% %     nInterval = floor(Npts/nColPts);
% %     
% %     setup.mesh.colpoints = nColPts*ones(1,nInterval);
% %     setup.mesh.phase(1).fraction = ones(1,nInterval)/nInterval;
    
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
setup.mesh.tolerance = 1e-2;
setup.mesh.maxiteration = 20;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 15;
setup.method = 'RPMintegration';
setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = 1e-3;

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOPS2 ---------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;

%--------------------------------------------------------------------------%
%------------------------------- Plot Solution ----------------------------%
%--------------------------------------------------------------------------%







    
% % %Save the solution if desired:
% % outputPrev = output;
% % save('oldSoln.mat','outputPrev');
