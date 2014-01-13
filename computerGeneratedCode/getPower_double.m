function Power = getPower_double(States, Actuators, Parameters)
%function Power = getPower_double(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_getPower_DoubleStance()
% 12-Jan-2014 20:02:06
%
%
% Matthew Kelly 
% Cornell University 
% 

x0 = States(:,1); % (m) Hip horizontal position wrt Foot One
y0 = States(:,2); % (m) Hip vertical position wrt Foot One
dx0 = States(:,3); % (m) Hip horizontal velocity
dy0 = States(:,4); % (m) Hip vertical velocity

F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two

x2 = Parameters.x2; % (m) Foot Two horizontal position wrt Foot One
y2 = Parameters.y2; % (m) Foot Two vertical position wrt Foot One

L1 = (x0.^2 + y0.^2).^(1./2); %(m) Leg One length
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2); %(m) Leg Two length
dL1 = (dx0.*x0 + dy0.*y0)./L1; %(m/s) Leg One length rate
dL2 = (dx0.*(x0 - x2) + dy0.*(y0 - y2))./L2; %(m/s) Leg Two length rate

Power = zeros(size(Actuators));
Power(:,1) = F1.*dL1; %legOne
Power(:,2) = F2.*dL2; %legTwo

end
