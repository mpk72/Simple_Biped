function output = Endpoint_DoubleStance(input)

P_dyn = input.auxdata.dynamics; 
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
        
Position_Start = kinematics(input.phase(1).initialstate', P_dyn);
Position_End = kinematics(input.phase(1).finalstate', P_dyn);

output.eventgroup(1).event = Position_Start.hip';
output.eventgroup(2).event = Position_End.hip';
        
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%        

%The objective function to be minimized is the cost of transport (CoT). 
%   CoT = Work / (Weight*Distance);
work = input.phase(1).integral;
weight = P_dyn.g*(P_dyn.m1 + P_dyn.m2 + P_dyn.M);
verySmallNumber = 1e-6;   %(m) Prevent divide by zero errors
distance = Position_Start.hip(1) - Position_End.hip(1);
%May want to be able to walk backwards, so need absolute value. 
distance = SmoothAbs(distance, P_cost.smoothing.distance) + verySmallNumber;
output.objective = work/(weight*distance); %Cost of Transport


end