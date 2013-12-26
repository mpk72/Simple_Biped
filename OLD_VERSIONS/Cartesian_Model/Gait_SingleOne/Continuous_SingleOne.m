function output = Continuous_SingleOne(input)

%Define Parameters
P_dyn = input.auxdata.dynamics;  
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Phase = 'S1';
stateDat = input.phase(1).state; 
actDat = input.phase(1).control;
phaseDat = input.auxdata.constant;
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
output(1).dynamics = DataRestructure(dStates,phaseDat);
output(1).path = packConstraints(pathCst,Phase);
output(1).integrand = costFunction(States, Actuators, Phase, P_cost); 

end