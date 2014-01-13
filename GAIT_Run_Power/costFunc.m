function [cost, path, info] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, C, AbsPos, AbsNeg, Phase, Step_Dist)

switch Phase
    case 'D'   
        Power = getPower_double(States, Actuators, P_Dyn);
        
    case 'S1'
        Power = getPower_single(States, Actuators);
        
    case 'F' 
        Power = getPower_flight(States, Actuators);
        
    otherwise
        error('Invalid Phase for cost function')
end

path = Power - (AbsPos-AbsNeg);

Act = Actuators;
for i=1:size(Act,2)
    Act(:,i) = Act(:,i)*C.scale.single_torque(i);
end
ActRate = Actuator_Rate;
for i=1:size(ActRate,2)
    ActRate(:,i) = ActRate(:,i)*C.scale.single_rate(i);
end

alpha = C.weight.actuator;
beta = C.weight.actuator_rate;

absX = sum(AbsPos+AbsNeg,2);  %abs(x) cost function
actSquared = alpha*sum(Act.^2,2);   %torque squared regularization
rateSquared = beta*sum(ActRate.^2,2);  %torque rate squared regularization

switch C.method
    case 'Work'
        cost = absX + actSquared + rateSquared;
    case 'CoT'
        %%%% HACK %%%%  Should be Step_Length, and needs scaling parameters
        cost = (absX + actSquared + rateSquared)./Step_Dist;
        %%%% DONE %%%%
    otherwise
        error('Unsupported cost function')
end

if nargout==3
    %provide additional information for analysis
    info.absX = absX;
    info.actSquared = actSquared;
    info.rateSquared = rateSquared;
end


end