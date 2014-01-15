function Power = getPower_single(States, Actuators)
%function Power = getPower_single(States, Actuators)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics_DoubleStance()
% 15-Jan-2014 13:47:06
%
%
% Matthew Kelly 
% Cornell University 
% 

th1 = States(:,1); % (rad) Leg One absolute angle
th2 = States(:,2); % (rad) Leg Two absolute angle
L1 = States(:,3); % (m) Leg One length
L2 = States(:,4); % (m) Leg Two length
dth1 = States(:,5); % (rad/s) Leg One absolute angular rate
dth2 = States(:,6); % (rad/s) Leg Two absolute angular rate
dL1 = States(:,7); % (m/s) Leg One extension rate
dL2 = States(:,8); % (m/s) Leg Two extensioin rate

F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
Thip = Actuators(:,4); % (Nm) Hip torque applied to Leg Two from Leg One

Power = zeros(size(Actuators));
Power(:,1) = F1.*dL1; %legOne
Power(:,2) = F2.*dL2; %legTwo
Power(:,3) = T1.*dth1; %ankleOne
Power(:,4) = -Thip.*(dth1 - dth2); %hip

end
