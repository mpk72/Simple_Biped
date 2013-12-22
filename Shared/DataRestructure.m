function [States, Actuators] = DataRestructure(stateDat,phaseDat,actDat)

% This function takes partial state and actuator data (as is used in GPOPS)
% and converts it to the full state and full actuation matrix for the
% dynamics function. The reverse conversion is also possible.

if strcmp(phaseDat.mode,'gpops_to_dynamics')
    switch phaseDat.phase
        case 'D'
            S = convertPartial(stateDat,phaseDat.phase);
            ONES = ones(size(S.x0));
            S.x1 = phaseDat.x1*ONES;
            S.y1 = phaseDat.y1*ONES;
            S.x2 = phaseDat.x2*ONES;
            S.y2 = phaseDat.y2*ONES;
            
            S.dx1 = phaseDat.dx1*ONES;
            S.dy1 = phaseDat.dy1*ONES;
            S.dx2 = phaseDat.dx2*ONES;
            S.dy2 = phaseDat.dy2*ONES;
            
            States = convert(S);
            
            Actuators = actDat;
            
        case 'S1'
            S = convertPartial(stateDat,phaseDat.phase);
            S.x1 = phaseDat.x1*ones(size(S.x0));
            S.y1 = phaseDat.y1*ones(size(S.x0));
            S.dx1 = zeros(size(S.x0));
            S.dy1 = zeros(size(S.x0));
            States = convert(S);
            
            A = convertPartial(actDat,phaseDat.phase);
            A.T2 = zeros(size(A.F1));
            Actuators = convert(A);
            
        case 'S2'
            S = convertPartial(stateDat,phaseDat.phase);
            S.x2 = phaseDat.x2*ones(size(S.x0));
            S.y2 = phaseDat.y2*ones(size(S.x0));
            S.dx2 = zeros(size(S.x0));
            S.dy2 = zeros(size(S.x0));
            States = convert(S);
            
            A = convertPartial(actDat,phaseDat.phase);
            A.T1 = zeros(size(A.F1));  %Should be no torque
            Actuators = convert(A);
            
        otherwise
            error('Other phases of motion not yet supported')
    end
elseif strcmp(phaseDat.mode,'dynamics_to_gpops')
    
    S = convert(stateDat);
    States = convertPartial(S,phaseDat.phase);
    if nargout == 2  %Don't calculate if not needed
        A = convert(actDat);
        Actuators = convertPartial(A,phaseDat.phase);
    end

else
    error('Reverse conversion not yet supported')
end

end