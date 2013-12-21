function cost = costFunction(States, Actuators, Phase, P)

%the cost for this model is just a smoothed version of the absolute value
%of the power going to the actuators. There is some fancy code to allow
%negative power to be counted in different ways.

dim = 2;   %Which dimension to sum the cost across

switch P.method
    case 'CoT'
        
        Power = actuatorPower(States, Actuators, Phase);
        
        PowerMatrix = [ Power.legOne,...
            Power.legTwo,...
            Power.ankleOne,...
            Power.ankleTwo,...
            Power.hip];
        
        alpha = P.smoothing.power;
        mUpp = 1;  %Cost of positive work
        mLow = P.negativeWorkCost; %Cost of negative work
        cost = sum(SmoothAbsFancy(PowerMatrix,alpha,mLow,mUpp),dim);
        
    case 'Squared'
        
        Act = convert(Actuators);
        d = P.pinionRadius;   %Pretend that the force is created by a torque motor with a rack and pinion
        
        cost =  (Act.F1*d).^2 + ...
                (Act.F2*d).^2 + ...
                Act.T1.^2 + ...
                Act.T2.^2 + ...
                Act.Thip.^2;
        
    otherwise
        error('Invalid cost metric')
end

end