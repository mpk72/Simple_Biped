function output = Continuous_Walking(input)

%Define Parameters
P_dyn = input.auxdata.dynamics;  
P_cost = input.auxdata.cost;

Slope = input.auxdata.misc.Ground_Slope;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Phase = 'D'; iphase = 1;
Step_Length = input.phase(iphase).parameter;
stateDat = input.phase(iphase).state; 
actDat = input.phase(iphase).control;
phaseDat.x1 = Step_Length*cos(Slope);
phaseDat.y1 = Step_Length*sin(Slope);
phaseDat.x2 = zeros(size(Step_Length));
phaseDat.y2 = zeros(size(Step_Length));
phaseDat.phase = Phase;
phaseDat.mode = 'gpops_to_dynamics';
[States, Actuators] = DataRestructure(stateDat,phaseDat,actDat);

[dStates, contactForces] = dynamics(States, Actuators ,P_dyn, Phase);
Kinematics = kinematics(States);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.footTwoContactAngle = atan2(contacts.H2, contacts.V2);
pathCst.legOneLength = Kinematics.L1;
pathCst.legTwoLength = Kinematics.L2;

phaseDat.mode = 'dynamics_to_gpops';
output(iphase).dynamics = DataRestructure(dStates,phaseDat);
output(iphase).path = packConstraints(pathCst,Phase);
output(iphase).integrand = costFunction(States, Actuators, Phase, P_cost); 


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Phase = 'S1'; iphase = 2;
Step_Length = input.phase(iphase).parameter;
stateDat = input.phase(iphase).state; 
actDat = input.phase(iphase).control;
phaseDat.x1 = Step_Length*cos(Slope);
phaseDat.y1 = Step_Length*sin(Slope);
phaseDat.phase = Phase;
phaseDat.mode = 'gpops_to_dynamics';
[States, Actuators] = DataRestructure(stateDat,phaseDat,actDat);

[dStates, contactForces] = dynamics(States, Actuators ,P_dyn, Phase);

Kinematics = kinematics(States);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.legOneLength = Kinematics.L1;
pathCst.legTwoLength = Kinematics.L2;

phaseDat.mode = 'dynamics_to_gpops';
output(iphase).dynamics = DataRestructure(dStates,phaseDat);
output(iphase).path = packConstraints(pathCst,Phase);
output(iphase).integrand = costFunction(States, Actuators, Phase, P_cost); 

end