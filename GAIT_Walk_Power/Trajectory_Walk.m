function [OUTPUT, plotInfo] = Trajectory_Walk(INPUT)

% addpath ../computerGeneratedCode; addpath ../Shared;

LOW = 1; UPP = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        START --  USER DEFINED                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Configuration parameters:
LEG_LENGTH = INPUT.physical.leg_length;
DURATION_SINGLE = INPUT.constraint.duration_single_stance;
DURATION_DOUBLE = INPUT.constraint.duration_double_stance;
SPEED = INPUT.constraint.speed;
MASS = INPUT.physical.totel_mass; 
GRAVITY = INPUT.physical.gravity;

%Common optimization parameters:
SOLVER = INPUT.optimize.solver;
TOL_OPT = INPUT.optimize.tol_opt;
TOL_MESH = INPUT.optimize.tol_mesh;
MAX_MESH_ITER = INPUT.optimize.max_mesh_iter;
auxdata.cost.weight.actuator = INPUT.cost.actuator_weight;
auxdata.cost.weight.actuator_rate = INPUT.cost.actuator_rate_weight;
auxdata.cost.method = INPUT.cost.method;

%Ground and step calculations
SLOPE = INPUT.constraint.ground_slope;    %Ground slope
CURVATURE = INPUT.constraint.ground_curvature;   %Ground curvature 
STEP_DIST = INPUT.constraint.step_distance;   %Horizontal component of the step vector
groundFunc = @(x,d)ground(x,SLOPE,CURVATURE,d,INPUT.constraint.center_clearance);
auxdata.ground.func = groundFunc;
auxdata.ground.clearance = INPUT.constraint.center_clearance;

%enforce friction cone at the contacts
CoeffFriction = INPUT.physical.coeff_friction;  %Between the foot and the ground
BndContactAngle = atan(CoeffFriction)*[-1;1]; %=atan2(H,V);
auxdata.ground.normal.bounds = BndContactAngle;

%Cartesian problem bounds:
DOMAIN = 1.25*STEP_DIST(UPP)*[-1;1];
gnd = groundFunc(linspace(DOMAIN(1),DOMAIN(2),1000),[]);
RANGE = [min(gnd); max(gnd)+2*LEG_LENGTH(UPP)];

%Store phase information
auxdata.phase = {'D','S1'};

%Physical parameters
hip_mass_fraction = INPUT.physical.hip_mass_fraction;
auxdata.dynamics.m = 0.5*(1-hip_mass_fraction)*MASS;   %(kg) foot mass
auxdata.dynamics.M = hip_mass_fraction*MASS;    %(kg) hip Mass
auxdata.dynamics.g = GRAVITY;   %(m/s^2) Gravitational acceleration

%Actuator Limits
Actuator_Time_Constant = 0.8;   %How quickly it can change from zero to max
Ank_Max = INPUT.physical.actuator_ank_saturate*LEG_LENGTH(UPP)*MASS*GRAVITY;
Hip_Max = INPUT.physical.actuator_hip_saturate*LEG_LENGTH(UPP)*MASS*GRAVITY;
Leg_Max = INPUT.physical.actuator_leg_saturate*MASS*GRAVITY;
Ank_Max_Rate = Ank_Max/Actuator_Time_Constant;
Hip_Max_Rate = Hip_Max/Actuator_Time_Constant;
Leg_Max_Rate = Leg_Max/Actuator_Time_Constant;

%COST FUNCTION:
if Leg_Max==0 || Ank_Max==0 || Hip_Max==0
    error('Scaling requires non-zero actuator saturation!')
end
auxdata.cost.scale.single_torque = 1./[...
    Leg_Max, Leg_Max, Ank_Max, Hip_Max];
auxdata.cost.scale.double_torque = 1./[...
    Leg_Max, Leg_Max];
auxdata.cost.scale.single_rate = ...
    Actuator_Time_Constant*auxdata.cost.scale.single_torque;
auxdata.cost.scale.double_rate = ...
    Actuator_Time_Constant*auxdata.cost.scale.double_torque;

%For animation only:
%1 = Real time, 0.5 = slow motion, 2.0 = fast forward
auxdata.animation.timeRate = 0.2;

switch auxdata.cost.method
    case 'Work'
        Max_Integrand = 200;
    case 'CoT'
        Max_Integrand = 200;
    otherwise
        error('Invalid Cost Function')
end

%Load the previous solution, if available
if INPUT.io.loadPrevSoln
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

P.Bnd(iphase).States = zeros(2,6);
P.Bnd(iphase).States(:,1) = DOMAIN; % (m) Hip horizontal position
P.Bnd(iphase).States(:,2) = RANGE; % (m) Hip vertical position One
P.Bnd(iphase).States(:,3) = 5*[-1;1]; % (m) Hip horizontal velocity
P.Bnd(iphase).States(:,4) = 2*[-1;1]; % (m) Hip vertical velocity
P.Bnd(iphase).States(:,5) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).States(:,6) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg Two

P.Bnd(iphase).Actuators = zeros(2,6);
P.Bnd(iphase).Actuators(:,1) = Leg_Max_Rate*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).Actuators(:,2) = Leg_Max_Rate*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd(iphase).Actuators(:,3) = Max_Integrand*[0;1]; % (N) abs(power(legOne))  --  POS
P.Bnd(iphase).Actuators(:,4) = Max_Integrand*[0;1]; % (N) abs(power(legTwo))  --  POS
P.Bnd(iphase).Actuators(:,5) = Max_Integrand*[0;1]; % (N) abs(power(legOne))  --  NEG
P.Bnd(iphase).Actuators(:,6) = Max_Integrand*[0;1]; % (N) abs(power(legTwo))  --  NEG

P.Bnd(iphase).Path = zeros(2,4);
P.Bnd(iphase).Path(:,1) = BndContactAngle; 
P.Bnd(iphase).Path(:,2) = BndContactAngle;
%Columns 3:4 are for the absolute value constraints

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
if INPUT.io.loadPrevSoln  
    iphase=1;
    
    guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;
else
    
    %Use default (bad) guess
    
    InitialStates = zeros(1,6);
    InitialStates(:,1) = -0.6*STEP_DIST(1); % (m) Hip horizontal position wrt Foot One
    InitialStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
    
    FinalStates = zeros(1,6);
    FinalStates(:,1) = -0.3*STEP_DIST(1); % (m) Hip horizontal position wrt Foot One
    FinalStates(:,2) = 0.8*LEG_LENGTH(UPP); % (m) Hip vertical position wrt Foot One
    
    guess.phase(iphase).time = [0; mean(P.Bnd(iphase).Duration)];
    
    guess.phase(iphase).state = [InitialStates ; FinalStates ];
    
    meanControl = 0.5*(bounds.phase(iphase).control.lower + bounds.phase(iphase).control.upper);
    guess.phase(iphase).control = [meanControl; meanControl];
    
    meanIntegral = 0.5*(bounds.phase(iphase).integral.lower + bounds.phase(iphase).integral.upper);
    guess.phase(iphase).integral = meanIntegral;
    
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

P.Bnd(iphase).States = zeros(2,12);
P.Bnd(iphase).States(:,1) = (pi/2)*[-1;1]; % (rad) Leg One absolute angle
P.Bnd(iphase).States(:,2) = (pi/2)*[-1;1]; % (rad) Leg Two absolute angle
P.Bnd(iphase).States(:,3) = LEG_LENGTH; % (m) Leg One length
P.Bnd(iphase).States(:,4) = LEG_LENGTH; % (m) Leg Two length
P.Bnd(iphase).States(:,5) = (pi/0.5)*[-1;1]; % (rad/s) Leg One absolute angular rate
P.Bnd(iphase).States(:,6) = (pi/0.5)*[-1;1]; % (rad/s) Leg Two absolute angular rate
P.Bnd(iphase).States(:,7) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg One extension rate
P.Bnd(iphase).States(:,8) = (diff(LEG_LENGTH)/0.3)*[-1;1]; % (m/s) Leg Two extensioin rate
P.Bnd(iphase).States(:,9) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).States(:,10) = Leg_Max*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd(iphase).States(:,11) = Ank_Max*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd(iphase).States(:,12) = Hip_Max*[-1;1]; % (Nm) Hip torque applied to Leg Two from

P.Bnd(iphase).Actuators = zeros(2,12);
P.Bnd(iphase).Actuators(:,1) = Leg_Max_Rate*[-1;1]; % (N) Compresive axial force in Leg One
P.Bnd(iphase).Actuators(:,2) = Leg_Max_Rate*[-1;1]; % (N) Compresive axial force in Leg Two
P.Bnd(iphase).Actuators(:,3) = Ank_Max_Rate*[-1;1]; % (Nm) External torque applied to Leg One
P.Bnd(iphase).Actuators(:,4) = Hip_Max_Rate*[-1;1]; % (Nm) Hip torque applied to Leg Two from Leg One
P.Bnd(iphase).Actuators(:,5) = Max_Integrand*[0;1]; % (N) abs(power(legOne))  --  POS
P.Bnd(iphase).Actuators(:,6) = Max_Integrand*[0;1]; % (N) abs(power(legTwo))  --  POS
P.Bnd(iphase).Actuators(:,7) = Max_Integrand*[0;1]; % (N) abs(power(ankle))  --  POS
P.Bnd(iphase).Actuators(:,8) = Max_Integrand*[0;1]; % (N) abs(power(hip))  --  POS
P.Bnd(iphase).Actuators(:,9) = Max_Integrand*[0;1]; % (N) abs(power(legOne))  --  NEG
P.Bnd(iphase).Actuators(:,10) = Max_Integrand*[0;1]; % (N) abs(power(legTwo))  --  NEG
P.Bnd(iphase).Actuators(:,11) = Max_Integrand*[0;1]; % (N) abs(power(ankle))  --  NEG
P.Bnd(iphase).Actuators(:,12) = Max_Integrand*[0;1]; % (N) abs(power(hip))  --  NEG

P.Bnd(iphase).Path = zeros(2,6);
P.Bnd(iphase).Path(:,1) = BndContactAngle; %Stance foot friction cone
P.Bnd(iphase).Path(:,2) = [0; LEG_LENGTH(UPP)]; %Swing foot clears ground
%Columns 3:6 are for abs() constraints

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
if INPUT.io.loadPrevSoln   
       guess.phase(iphase).state = outputPrev.result.solution.phase(iphase).state;
    guess.phase(iphase).control = outputPrev.result.solution.phase(iphase).control;
    guess.phase(iphase).integral = outputPrev.result.solution.phase(iphase).integral;
    guess.phase(iphase).time = outputPrev.result.solution.phase(iphase).time;

else %Use default (bad) guess
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

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Parameters                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
bounds.parameter.lower = STEP_DIST(LOW);
bounds.parameter.upper = STEP_DIST(UPP);
guess.parameter = mean(STEP_DIST);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Event Group                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Continuous hip motion
bounds.eventgroup(1).lower = zeros(1,4);
bounds.eventgroup(1).upper = zeros(1,4);

%Hip translation and velocity defect
bounds.eventgroup(2).lower = zeros(1,4); 
bounds.eventgroup(2).upper = zeros(1,4);

% initial swing foot speed
bounds.eventgroup(3).lower = zeros(1,2);
bounds.eventgroup(3).upper = zeros(1,2);

%Swing foot targets:
bounds.eventgroup(4).lower = zeros(1,4); 
bounds.eventgroup(4).upper = zeros(1,4);

%Speed
bounds.eventgroup(5).lower = SPEED(LOW);
bounds.eventgroup(5).upper = SPEED(UPP);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Mesh Parameters                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSections = 5;  %Number of sections
nColPts = 8;  %Collocation points per phase
mesh.phase(1).colpoints = nColPts*ones(1,nSections);
mesh.phase(1).fraction  = ones(1,nSections)/nSections;
mesh.phase(2).colpoints = nColPts*ones(1,nSections);
mesh.phase(2).fraction  = ones(1,nSections)/nSections;
mesh.colpointsmin = 5;
mesh.colpointsmax = 20;
mesh.method = 'hp'; % {'hp','hp1'};
mesh.tolerance = TOL_MESH;
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
setup.nlp.solver = SOLVER; %{'snopt', 'ipopt'};
setup.derivatives.supplier = 'sparseCD'; %{'sparseBD', 'sparseFD', 'sparseCD'}
setup.derivatives.derivativelevel = 'second'; %{'first','second'};
setup.method = 'RPMintegration';
%setup.scales.method = 'automatic-bounds';
setup.nlp.options.tolerance = TOL_OPT;

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Output (Save, Plot, Animation)                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if INPUT.io.createPlots || INPUT.io.runAnimation || nargout==2
    plotInfo = getPlotInfo(output);
end
if INPUT.io.createPlots
    figNums = 2:10;
    plotSolution(plotInfo,figNums);
end
if INPUT.io.runAnimation
    figNum = 1;
    animation(plotInfo,figNum);
end

if (strcmp(SOLVER,'ipopt') && output.result.nlpinfo==0) ||...
        (strcmp(SOLVER,'snopt') && output.result.nlpinfo==1)
    if INPUT.io.saveSolution
        %Then successful  --  Save the solution:
        outputPrev = output;
        save(['oldSoln_' auxdata.cost.method '.mat'],'outputPrev');
    end
end

OUTPUT = output;

end



