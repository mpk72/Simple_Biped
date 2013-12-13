function output = Continuous_Walking(input)

%Define Parameters
P_dyn = input.auxdata.dynamics;  
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

States = input.phase(1).state';  
Actuators = input.phase(1).control';
Phase = 'D';

[dStates, contactForces] = dynamics_doubleStance(States, Actuators ,P_dyn);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.footTwoContactAngle = atan2(contacts.H2, contacts.V2);

output(1).dynamics = dStates';
output(1).path = packConstraints(pathCst,Phase)';
output(1).integrand = costFunction(States, Actuators, Phase, P_cost)'; 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

States = input.phase(2).state';  
Actuators = input.phase(2).control';
Phase = 'S1';

[dStates, contactForces] = dynamics_singleStanceOne(States, Actuators,P_dyn);
Position = kinematics(States, P_dyn);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.footTwoHeight = Position.footTwo(2,:);

output(2).dynamics = dStates';
output(2).path = packConstraints(pathCst,Phase)';
output(2).integrand = costFunction(States, Actuators, Phase, P_cost)'; 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 3  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

States = input.phase(3).state';  
Actuators = input.phase(3).control';
Phase = 'D';

[dStates, contactForces] = dynamics_doubleStance(States, Actuators,P_dyn);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.footTwoContactAngle = atan2(contacts.H2, contacts.V2);

output(3).dynamics = dStates';
output(3).path = packConstraints(pathCst,Phase)';
output(3).integrand = costFunction(States, Actuators, Phase, P_cost)'; 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 4  --  S2  --  Single Stance Two                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

States = input.phase(4).state';  
Actuators = input.phase(4).control';
Phase = 'S2';

[dStates, contactForces] = dynamics_singleStanceTwo(States, Actuators,P_dyn);
Position = kinematics(States, P_dyn);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneHeight = Position.footOne(2,:);
pathCst.footTwoContactAngle = atan2(contacts.H2, contacts.V2);

output(4).dynamics = dStates';
output(4).path = packConstraints(pathCst,Phase)';
output(4).integrand = costFunction(States, Actuators, Phase, P_cost)'; 

end