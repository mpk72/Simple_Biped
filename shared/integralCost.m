function C = integralCost(States, Actuators, Phase, Parameters)
%INTEGRALCOST computes the integral cost for the simple biped
% For now, there are two supported cost functions. The first is a simple
% actuation squared function, and the second is a modified version of cost
% of transport

switch Phase
    case 'S1'  %Foot one is in contact with ground, at origin
        
        % x0 = States(:,1); %(m) Hip horizontal position
        % y0 = States(:,2); %(m) Hip vertical position
        % x2 = States(:,3); %(m) Foot Two horizontal position
        % y2 = States(:,4); %(m) Foot Two vertical position
        dx0 = States(:,5); %(m/s) Hip horizontal velocity
        dy0 = States(:,6); %(m/s) Hip vertical velocity
        dx2 = States(:,7); %(m/s) Foot Two horizontal velocity
        dy2 = States(:,8); %(m/s) Foot Two vertical velocity
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        H2 = Actuators(:,3); %(N) Horizontal force on hip from Leg Two
        V2 = Actuators(:,4); %(N) Vertical force on hip from Leg Two
        
        % M = Parameters.M;  %(kg) hip mass
        % m1 = Parameters.m1; %(kg) foot one mass
        % m2 = Parameters.m2; %(kg) foot two mass
        % g = Parameters.g;  %(m/s^2) gravity
        
        switch Parameters.costMethod
            case 'CoT2' %Cost of transport, squared, constants removed
                X2 = Parameters.StepVector(:,1);
                Y2 = Parameters.StepVector(:,2);
                power2 = (dx0.*(H1+H2)).^2 + (dy0.*(V1+V2)).^2 + ...
                    (dx2.*H2).^2 + (dy2.*V2).^2;
                dist2 = X2.^2 + Y2.^2;
                C = power2/dist2;
            case 'Squared' %Sum force squared
                C = H1.^2 + H2.^2 + V1.^2 + V2.^2;
            otherwise
                error('Invalid cost method')
        end
        
    case 'D'  %Both feet on ground, Foot one at origin
        
        % x0 = States(:,1); %(m) Hip horizontal position
        % y0 = States(:,2); %(m) Hip vertical position
        dx0 = States(:,3); %(m/s) Hip horizontal velocity
        dy0 = States(:,4); %(m/s) Hip vertical velocity
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        H2 = Actuators(:,3); %(N) Horizontal force on hip from Leg Two
        V2 = Actuators(:,4); %(N) Vertical force on hip from Leg Two
        
        % M = Parameters.M;  %(kg) hip mass
        % g = Parameters.g;  %(m/s^2) gravity
        
        switch Parameters.costMethod
            case 'CoT2' %Cost of transport, squared, constants removed
                X2 = Parameters.StepVector(:,1);
                Y2 = Parameters.StepVector(:,2);
                power2 = (dx0.*(H1+H2)).^2 + (dy0.*(V1+V2)).^2;
                dist2 = X2.^2 + Y2.^2;
                C = power2/dist2;
            case 'Squared' %Sum force squared
                C = H1.^2 + H2.^2 + V1.^2 + V2.^2;
            otherwise
                error('Invalid cost method')
        end
        
        
    otherwise
        error('Invalid Phase')
end

end