function Cst = constraints(States, Actuators, Phase, Parameters)
%CONSTRAINTS computes the constraint functions for the simple biped

switch Phase
    case 'S1'  %Foot one is in contact with ground, at origin
        
        x0 = States(:,1); %(m) Hip horizontal position
        y0 = States(:,2); %(m) Hip vertical position
        x2 = States(:,3); %(m) Foot Two horizontal position
        y2 = States(:,4); %(m) Foot Two vertical position
        %        dx0 = States(:,5); %(m/s) Hip horizontal velocity
        %        dy0 = States(:,6); %(m/s) Hip vertical velocity
        %        dx2 = States(:,7); %(m/s) Foot Two horizontal velocity
        %        dy2 = States(:,8); %(m/s) Foot Two vertical velocity
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        % H2 = Actuators(:,3); %(N) Horizontal force on hip from Leg Two
        % V2 = Actuators(:,4); %(N) Vertical force on hip from Leg Two
        
        % M = Parameters.M;  %(kg) hip mass
        m1 = Parameters.m1; %(kg) foot one mass
        %  m2 = Parameters.m2; %(kg) foot two mass
        g = Parameters.g;  %(m/s^2) gravity
        
        Cst = zeros(length(x0),3);
        
        %Stance foot is at origin by definition
        x1 = 0;
        y1 = 0;
        
        %Leg One Length Squared
        Dx1 = x0-x1;
        Dy1 = y0-y1;
        Cst(:,1) = Dx1.^2 + Dy1.^2;
        
        %Leg Two Length Squared
        Dx2 = x0-x2;
        Dy2 = y0-y2;
        Cst(:,2) = Dx2.^2 + Dy2.^2;
        
        %Foot One, Contact force angle (Friction cone)
        Cst(:,3) = atan2(H1,V1-m1*g);
        
    case 'D'  %Both feet on ground, Foot one at origin
        
        x0 = States(:,1); %(m) Hip horizontal position
        y0 = States(:,2); %(m) Hip vertical position
        
        H1 = Actuators(:,1); %(N) Horizontal force on hip from Leg One
        V1 = Actuators(:,2); %(N) Vertical force on hip from Leg One
        H2 = Actuators(:,3); %(N) Horizontal force on hip from Leg Two
        V2 = Actuators(:,4); %(N) Vertical force on hip from Leg Two
        
        % M = Parameters.M;  %(kg) hip mass
        m1 = Parameters.m1; %(kg) foot one mass
        m2 = Parameters.m2; %(kg) foot two mass
        g = Parameters.g;  %(m/s^2) gravity
       
        %Stance foot is at origin by definition
        x1 = 0;
        y1 = 0;
        
        X2 = Parameters.StepVector(:,1); 
        Y2 = Parameters.StepVector(:,2);
        
        Cst = zeros(length(x0),4);
        
        %Leg One Length Squared
        Dx1 = x0-x1;
        Dy1 = y0-y1;
        Cst(:,1) = Dx1.^2 + Dy1.^2;
        
        %Leg Two Length Squared
        Dx2 = x0-X2;
        Dy2 = y0-Y2;
        Cst(:,2) = Dx2.^2 + Dy2.^2;
        
        %Foot One, Contact force angle (Friction cone)
        Cst(:,3) = atan2(H1,V1-m1*g);
        
        %Foot Two, Contact force angle (Friction cone)
        Cst(:,4) = atan2(H2,V2-m2*g);
        
    otherwise
        error('Invalid Phase')
end



end