function output = Endpoint_Walking(input)
stepVec = input.auxdata.parameters.StepVector;

%Require that things be monotonic in time
output.eventgroup(1).event = input.phase(2).initialtime - input.phase(1).finaltime;

%Hip defect
output.eventgroup(2).event = input.phase(1).finalstate - input.phase(2).initialstate([1:2,5:6]); 

%Swing foot defect
tmpState = [...
            -stepVec(1),...
            -stepVec(2),...
            0,...
            0,...
            ];       
output.eventgroup(3).event = tmpState - input.phase(2).initialstate([3:4,7:8]);    

%Step vector satisfied
tmpState = [...
            stepVec(1),...
            stepVec(2),...
            ];
output.eventgroup(4).event = tmpState - input.phase(2).finalstate(3:4);    

%Periodic 
target = input.phase(1).initialstate;
target(1) = target(1) + stepVec(1);
target(2) = target(2) + stepVec(2);
output.eventgroup(5).event = target - input.phase(2).finalstate([1:2,5:6]);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
output.objective = input.phase(1).integral + input.phase(2).integral;

end