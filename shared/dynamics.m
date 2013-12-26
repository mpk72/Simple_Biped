function dStates = dynamics(States, Actuators, Phase, Parameters)
%DYNAMICS computes the dynamics for a simple biped
%   This function computes the first time derivative of the biped states,
%   given the phase of motion and actuation. For detailed information about
%   the inputs, please refer to the source code below, which unpacks the
%   inputs into aptly named variables.

switch Phase
    case 'S1'  %Foot one is in contact with ground, at origin
        
        %        x0 = States(:,1); %(m) Hip horizontal position
        %        y0 = States(:,2); %(m) Hip vertical position
        %        x2 = States(:,3); %(m) Foot Two horizontal position
        %        y2 = States(:,4); %(m) Foot Two vertical position
        %        dx0 = States(:,5); %(m/s) Hip horizontal velocity
        %        dy0 = States(:,6); %(m/s) Hip vertical velocity
        %        dx2 = States(:,7); %(m/s) Foot Two horizontal velocity
        %        dy2 = States(:,8); %(m/s) Foot Two vertical velocity
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        H2 = Actuators(:,4); %(N) Horizontal force on hip from Leg Two
        V2 = Actuators(:,3); %(N) Vertical force on hip from Leg Two
        
        M = Parameters.M;  %(kg) hip mass
        % m1 = Parameters.m1; %(kg) foot one mass
        m2 = Parameters.m2; %(kg) foot two mass
        g = Parameters.g;  %(m/s^2) gravity
        
        dStates = zeros(size(States));
        
        ddx0 = (H1+H2)/M;
        ddy0 = (V1+V2)/M - g;
        ddx2 = -(H1+H2)/m2;
        ddy2 = -(V1+V2)/m2 - g;
        
        dStates(:,1:4) = States(:,5:8);  %First-order form (dx=v)
        dStates(:,5) = ddx0;
        dStates(:,6) = ddy0;
        dStates(:,7) = ddx2;
        dStates(:,8) = ddy2;
        
    case 'D'  %Both feet on ground, Foot one at origin
        
        %        x0 = States(:,1); %(m) Hip horizontal position
        %        y0 = States(:,2); %(m) Hip vertical position
        %        dx0 = States(:,3); %(m/s) Hip horizontal velocity
        %        dy0 = States(:,4); %(m/s) Hip vertical velocity
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        H2 = Actuators(:,4); %(N) Horizontal force on hip from Leg Two
        V2 = Actuators(:,3); %(N) Vertical force on hip from Leg Two
        
        M = Parameters.M;  %(kg) hip mass
        g = Parameters.g;  %(m/s^2) gravity
        
        dStates = zeros(size(States));
        
        ddx0 = (H1+H2)/M;
        ddy0 = (V1+V2)/M - g;
        
        dStates(:,1:2) = States(:,3:4);  %First-order form (dx=v)
        dStates(:,3) = ddx0;
        dStates(:,4) = ddy0;
        
    otherwise
        error('Invalid Phase')
end

end

