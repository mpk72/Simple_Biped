function output = Continuous_DoubleStance(input)

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

[Position, Velocity] = kinematics(States,P_dyn);

contacts = convert(contactForces);  %Make into a struct
pathCst.footOneContactAngle = atan2(contacts.H1, contacts.V1);
pathCst.footTwoContactAngle = atan2(contacts.H2, contacts.V2);
pathCst.hipHeight = Position.hip(1,:)';
pathCst.footOneVel = Velocity.footOne';
pathCst.footTwoVel = Velocity.footTwo';

output(1).dynamics = dStates';
output(1).path = packConstraints(pathCst,Phase)';
output(1).integrand = costFunction(States, Actuators, Phase, P_cost)'; 

end