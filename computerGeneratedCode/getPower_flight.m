function Power = getPower_flight(States, Actuators)
%function Power = getPower_flight(States, Actuators)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_getPower_Flight()
% 16-Jan-2014 13:16:36
%
%
% Matthew Kelly 
% Cornell University 
% 

x = States(:,1); % (m) Foot One Horizontal Position
y = States(:,2); % (m) Foot One Vertical Position
th1 = States(:,3); % (rad) Leg One absolute angle
th2 = States(:,4); % (rad) Leg Two absolute angle
L1 = States(:,5); % (m) Leg One length
L2 = States(:,6); % (m) Leg Two length
dx = States(:,7); % (m) Foot One Horizontal Velocity
dy = States(:,8); % (m) Foot One Vertical Velocity
dth1 = States(:,9); % (rad/s) Leg One absolute angular rate
dth2 = States(:,10); % (rad/s) Leg Two absolute angular rate
dL1 = States(:,11); % (m/s) Leg One extension rate
dL2 = States(:,12); % (m/s) Leg Two extensioin rate

F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
Thip = Actuators(:,3); % (Nm) Hip torque applied to Leg Two from Leg One

Power = zeros(size(Actuators));
Power(:,1) = F1.*dL1; %legOne
Power(:,2) = F2.*dL2; %legTwo
Power(:,3) = -Thip.*(dth1 - dth2); %hip

end
