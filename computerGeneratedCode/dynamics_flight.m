function dStates = dynamics_flight(States, Actuators, Parameters)
%function dStates = dynamics_flight(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Dynamics_Flight()
% 16-Jan-2014 13:16:34
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

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

dStates = zeros(size(States));
dStates(:,(1:6)) = States(:,(1+6):(6+6));
dStates(:,7) = -(Thip.*cos(th1) - F1.*L1.*sin(th1))./(L1.*m);
dStates(:,8) = -(Thip.*sin(th1) + L1.*g.*m + F1.*L1.*cos(th1))./(L1.*m);
dStates(:,9) = -(L2.*M.*Thip + L2.*Thip.*m - L1.*Thip.*m.*cos(th1 - th2) + F2.*L1.*L2.*m.*sin(th1 - th2) + 2.*L1.*L2.*M.*dL1.*dth1.*m)./(L1.^2.*L2.*M.*m);
dStates(:,10) = (L1.*M.*Thip + L1.*Thip.*m - L2.*Thip.*m.*cos(th1 - th2) + F1.*L1.*L2.*m.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m)./(L1.*L2.^2.*M.*m);
dStates(:,11) = (Thip.*m.*sin(th1 - th2) + F1.*L2.*M + F1.*L2.*m + F2.*L2.*m.*cos(th1 - th2) + L1.*L2.*M.*dth1.^2.*m)./(L2.*M.*m);
dStates(:,12) = (Thip.*m.*sin(th1 - th2) + F2.*L1.*M + F2.*L1.*m + F1.*L1.*m.*cos(th1 - th2) + L1.*L2.*M.*dth2.^2.*m)./(L1.*M.*m);

end
