function output = Continuous_Run(input)

%Check MAIN_Walk for index definitions throughout this file

groundFunc = input.auxdata.ground.func;
[~, n1] = feval(groundFunc,0,[]);   %Ground normal for stance foot

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Flight                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:12;
IdxAct = 13:15;
IdxActRate = 1:3;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
P_Dyn = input.auxdata.dynamics;

dStates = dynamics_flight(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

IdxAbsPos = 4:6;
IdxAbsNeg = 7:9;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

output(iphase).path = zeros(size(dStates,1),5);
Position = getPosVel_flight(States);
STEP_DIST = input.phase(iphase).parameter;
[~,~,min_height] = feval(groundFunc,Position.footOne.x,STEP_DIST);
output(iphase).path(:,1) = ... %Foot height above ground
    Position.footOne.y;% %%%% HACK %%%% - min_height;
[~,~,min_height] = feval(groundFunc,Position.footTwo.x,STEP_DIST);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y;% %%%% HACK %%%%  - min_height;

P_Cost = input.auxdata.cost;
[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'F', STEP_DIST);
output(iphase).path(:,3:5) = path;
output(iphase).integrand = cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

IdxState = 1:8;
IdxAct = 9:12;
IdxActRate = 1:4;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);

[dStates, contactForces] = dynamics_single(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1) + n1;

Position = getPosVel_single(States);
STEP_DIST = input.phase(iphase).parameter;
[~,~,min_height] = feval(groundFunc,Position.footTwo.x,STEP_DIST);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - min_height;

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'S1', STEP_DIST);
output(iphase).path(:,3:6) = path;
output(iphase).integrand = cost;

end