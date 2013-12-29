function output = Continuous_Walking(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Phase = 'D'; iphase = 1;
States = input.phase(iphase).state; 
Actuators = input.phase(iphase).control;
Parameters = input.auxdata.parameters;

output(iphase).dynamics = dynamics(States, Actuators, Phase, Parameters);
output(iphase).path = constraints(States, Actuators, Phase, Parameters);
output(iphase).integrand = integralCost(States, Actuators, Phase, Parameters); 


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Phase = 'S1'; iphase = 2;
States = input.phase(iphase).state; 
Actuators = input.phase(iphase).control;
Parameters = input.auxdata.parameters;

output(iphase).dynamics = dynamics(States, Actuators, Phase, Parameters);
output(iphase).path = constraints(States, Actuators, Phase, Parameters);
output(iphase).integrand = integralCost(States, Actuators, Phase, Parameters);

end