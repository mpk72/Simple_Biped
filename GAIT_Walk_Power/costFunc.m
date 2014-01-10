function [cost, path, info] = costFunc(States, Actuators, Actuator_Rate,...
    auxdata, AbsPos, AbsNeg, Phase)

C = auxdata.cost;

switch Phase
    case 'D'
        
        Power = getPower_double(States, Actuators, auxdata.dynamics);
        path = Power - (AbsPos-AbsNeg);
        
        Act = Actuators;
        for i=1:size(Act,2)
            Act(:,i) = Act(:,i)*C.scale.double_torque(i);
        end
        ActRate = Actuator_Rate;
        for i=1:size(ActRate,2)
            ActRate(:,i) = ActRate(:,i)*C.scale.double_rate(i);
        end
        
    case 'S1'
        Power = getPower_single(States, Actuators);
        path = Power - (AbsPos-AbsNeg);
        
        Act = Actuators;
        for i=1:size(Act,2)
            Act(:,i) = Act(:,i)*C.scale.single_torque(i);
        end
        ActRate = Actuator_Rate;
        for i=1:size(ActRate,2)
            ActRate(:,i) = ActRate(:,i)*C.scale.single_rate(i);
        end
    otherwise
        error('Invalid Phase for cost function')
end

switch C.method
    case 'Work'
        alpha = C.weight.actuator;
        beta = C.weight.actuator_rate;
        
        absX = sum(AbsPos+AbsNeg,2);  %abs(x) cost function
        actSquared = alpha*sum(Act.^2,2);   %torque squared regularization
        rateSquared = beta*sum(ActRate.^2,2);  %torque rate squared regularization
        
        cost = absX + actSquared + rateSquared;
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