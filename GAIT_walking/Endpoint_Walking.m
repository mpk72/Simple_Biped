function output = Endpoint_Walking(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Defect Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
stepVec = input.auxdata.parameters.StepVector;

% Double Stance -> Single Stance One
    minus_x0 = input.phase(1).finalstate(1); %(m) Hip horizontal position
    minus_y0 = input.phase(1).finalstate(2); %(m) Hip vertical position
    minus_x2 = stepVec(:,1); %(m) Foot Two horizontal position
    minus_y2 = stepVec(:,2); %(m) Foot Two vertical position
    minus_dx0 = input.phase(1).finalstate(3); %(m/s) Hip horizontal velocity
    minus_dy0 = input.phase(1).finalstate(4); %(m/s) Hip vertical velocity

    plus_x0 = input.phase(2).initialstate(1); %(m) Hip horizontal position
    plus_y0 = input.phase(2).initialstate(2); %(m) Hip vertical position
    plus_x2 = input.phase(2).initialstate(3); %(m) Foot Two horizontal position
    plus_y2 = input.phase(2).initialstate(4); %(m) Foot Two vertical position
    plus_dx0 = input.phase(2).initialstate(5); %(m/s) Hip horizontal velocity
    plus_dy0 = input.phase(2).initialstate(6); %(m/s) Hip vertical velocity
    
    output.eventgroup(1).event = [...
        plus_x0 - minus_x0,...
        plus_y0 - minus_y0,...
        plus_x2 - minus_x2,...
        plus_y2 - minus_y2,...
        plus_dx0 - minus_dx0,...
        plus_dy0 - minus_dy0,...
        ];
    
 % Single Stance One -> Double Stance (Periodic constraint)
    minus_x0 = input.phase(1).initialstate(1) - stepVec(:,1); %(m) Hip horizontal position
    minus_y0 = input.phase(1).initialstate(2) - stepVec(:,2); %(m) Hip vertical position
    minus_x2 = -stepVec(1); %(m) Foot Two horizontal position
    minus_y2 = -stepVec(2); %(m) Foot Two vertical position
    minus_dx0 = input.phase(1).initialstate(3); %(m/s) Hip horizontal velocity
    minus_dy0 = input.phase(1).initialstate(4); %(m/s) Hip vertical velocity

    plus_x0 = input.phase(2).finalstate(1); %(m) Hip horizontal position
    plus_y0 = input.phase(2).finalstate(2); %(m) Hip vertical position
    plus_x2 = input.phase(2).finalstate(3); %(m) Foot Two horizontal position
    plus_y2 = input.phase(2).finalstate(4); %(m) Foot Two vertical position
    plus_dx0 = input.phase(2).finalstate(5); %(m/s) Hip horizontal velocity
    plus_dy0 = input.phase(2).finalstate(6); %(m/s) Hip vertical velocity
    
    output.eventgroup(2).event = [...
        plus_x0 - minus_x0,...
        plus_y0 - minus_y0,...
        plus_x2 - minus_x2,...
        plus_y2 - minus_y2,...
        plus_dx0 - minus_dx0,...
        plus_dy0 - minus_dy0,...
        ];
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Other Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Require that things be monotonic in time
output.eventgroup(3).event = input.phase(2).initialtime - input.phase(1).finaltime;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
 output.objective = input.phase(1).integral + input.phase(2).integral;

end