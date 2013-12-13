function output = Endpoint_Walking(input)

P_dyn = input.auxdata.dynamics; 
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Defect between phase 1 and 2
output.eventgroup(1).event = input.phase(2).initialstate - ...
            phaseMap(input.phase(1).finalstate','D','S1')';
        
Position_1 = kinematics(input.phase(1).initialstate', P_dyn);

%For computing distance travelled
startPos = Position_1.CoM;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Defect between phase 2 and 3
output.eventgroup(2).event = input.phase(3).initialstate - ...
            phaseMap(input.phase(2).finalstate','S1','D')';

        
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 3  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Defect between phase 3 and 4
output.eventgroup(3).event = input.phase(4).initialstate - ...
            phaseMap(input.phase(3).finalstate','D','S2')';

        
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 4  --  S2  --  Single Stance Two                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Defect between phase 4 and 1
Defect_41 = input.phase(1).initialstate - ...
            phaseMap(input.phase(4).finalstate','S2','D')';

%Need to remove the periodic defect on horizontal translation:        
%%%% HACK %%%% Dec 11, 2013
% The defect for horizontal translation in the system is stored in the
% first index of the state. Pass through all defects except for this one.
% This will cause a bug if the order of the packing of the state changes.
output.eventgroup(4).event = Defect_41(:,2:12);
%%%% DONE %%%%

%For computing distance travelled
Position_4 = kinematics(input.phase(1).initialstate', P_dyn);
endPos = Position_4.CoM;        
   

        
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%        

%The objective function to be minimized is the cost of transport (CoT). 
%   CoT = Work / (Weight*Distance);
work = input.phase(1).integral +...
       input.phase(2).integral +...
       input.phase(3).integral +...
       input.phase(4).integral;
weight = P_dyn.g*(P_dyn.m1 + P_dyn.m2 + P_dyn.M);
verySmallNumber = 1e-6;   %(m) Prevent divide by zero errors
distance = endPos(1)-startPos(1);
%May want to be able to walk backwards, so need absolute value. 
distance = SmoothAbs(distance, P_cost.smoothing.distance) + verySmallNumber;
output.objective = work/(weight*distance); %Cost of Transport


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Other Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%  

%Require that things be monotonic in time
output.eventgroup(5).event = [...
    input.phase(2).initialtime - input.phase(1).finaltime;
    input.phase(3).initialtime - input.phase(2).finaltime;
    input.phase(4).initialtime - input.phase(3).finaltime];

%Require that the system moves at some horizontal speed
output.eventgroup(6).event = (endPos(1)-startPos(1))/...
    (input.phase(4).finaltime - input.phase(1).initialtime);

%Enforce a step length constraint:
output.eventgroup(7).event = endPos(1)-startPos(1);

end