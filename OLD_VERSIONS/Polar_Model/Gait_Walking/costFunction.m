function cost = costFunction(States, Actuators, Phase, P)

%the cost for this model is just a smoothed version of the absolute value
%of the power going to the actuators. There is some fancy code to allow
%negative power to be counted in different ways.

Power = actuatorPower(States, Actuators, Phase);

PowerMatrix = [ Power.legOne;
                Power.legTwo;
                Power.ankleOne;
                Power.ankleTwo;
                Power.hip];
                
alpha = P.smoothing.power;
mUpp = 1;  %Cost of positive work
mLow = P.negativeWorkCost; %Cost of negative work
dim = 1;  %Sum along the first dimension of the smoothed matrix
cost = sum(SmoothAbsFancy(PowerMatrix,alpha,mLow,mUpp),dim);

end