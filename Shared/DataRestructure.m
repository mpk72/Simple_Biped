function [States, Actuators] = DataRestructure(stateDat,phaseDat,actDat)

% This function takes partial state and actuator data (as is used in GPOPS)
% and converts it to the full state and full actuation matrix for the
% dynamics function. The reverse conversion is also possible.

if strcmp(phaseDat.mode,'gpops_to_dynamics')
    switch phaseDat.phase
        case 'D'
            S.x1 = phaseDat.x1;
            S.y1 = phaseDat.y1;
            S.x2 = phaseDat.x2;
            S.y2 = phaseDat.y2;
            
            S.dx1 = phaseDat.dx1;
            S.dy1 = phaseDat.dy1;
            S.dx2 = phaseDat.dx2;
            S.dy2 = phaseDat.dy2;
            
            S.x0 = stateDat(:,1);
            S.y0 = stateDat(:,2);
            S.dx0 = stateDat(:,3);
            S.dy0 = stateDat(:,4);
            
            States = convert(S);
            
            Actuators = actDat;
            
        otherwise
            error('Other phases of motion not yet supported')
    end
elseif strcmp(phaseDat.mode,'dynamics_to_gpops')
    switch phaseDat.phase
        case 'D'
            S = convert(stateDat);
            
            nStates = length(S.x0);
            States = zeros(nStates,4);
            
            States(:,1) = S.x0;
            States(:,2) = S.y0;
            States(:,3) = S.dx0;
            States(:,4) = S.dy0;
            
            Actuators = [];
        otherwise
            error('Other phases of motion not yet supported!')
    end
else
    error('Reverse conversion not yet supported')
end

end