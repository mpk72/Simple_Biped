function output = Continuous_SingleOne(input)

%Define Parameters
P_dyn = input.auxdata.dynamics;  
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

stateDat = input.phase(1).state; 
actDat = input.phase(1).control;
phaseDat = input.auxdata.constant;
phaseDat.phase = 'S1';
phaseDat.mode = 'gpops_to_dynamics';
[States, Actuators] = DataRestructure(stateDat,phaseDat,actDat);

[dStates, contactForces] = dynamics_singleStanceOne(States, Actuators ,P_dyn);

Kinematics = kinematics(States);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.legOneLength = Kinematics.L1;
pathCst.legTwoLength = Kinematics.L2;

phaseDat.mode = 'dynamics_to_gpops';
output(1).dynamics = DataRestructure(dStates,phaseDat);
output(1).path = packConstraints(pathCst,'S1');
output(1).integrand = costFunction(States, Actuators, 'S1', P_cost); 

end