function [dStates, contactForces] = dynamics_single(States, Actuators, Parameters)
%function [dStates, contactForces] = dynamics_single(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Dynamics_SingleStance()
% 15-Jan-2014 13:47:04
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

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

dStates = zeros(size(States));
dStates(:,(1:4)) = States(:,(1+4):(4+4));
dStates(:,5) = (L2.*T1 - L2.*Thip + L1.*Thip.*cos(th1 - th2) - F2.*L1.*L2.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL1.*dth1 + L1.*L2.*M.*g.*sin(th1))./(L1.^2.*L2.*M);
dStates(:,6) = (L1.*M.*Thip + L1.*Thip.*m + L2.*T1.*m.*cos(th1 - th2) - L2.*Thip.*m.*cos(th1 - th2) + F1.*L1.*L2.*m.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m)./(L1.*L2.^2.*M.*m);
dStates(:,7) = (Thip.*sin(th1 - th2) + F1.*L2 + F2.*L2.*cos(th1 - th2) + L1.*L2.*M.*dth1.^2 - L2.*M.*g.*cos(th1))./(L2.*M);
dStates(:,8) = (Thip.*m.*sin(th1 - th2) - T1.*m.*sin(th1 - th2) + F2.*L1.*M + F2.*L1.*m + F1.*L1.*m.*cos(th1 - th2) + L1.*L2.*M.*dth2.^2.*m)./(L1.*M.*m);

% contactForces(:,1) == H1 == (N) Foot One, horizontal contact force
% contactForces(:,2) == V1 == (N) Foot One, vertical contact force
contactForces = zeros(size(States,1),2);
contactForces(:,1) = -(T1.*cos(th1) - Thip.*cos(th1) + F1.*L1.*sin(th1))./L1;
contactForces(:,2) = (Thip.*sin(th1) - T1.*sin(th1) + L1.*g.*m + F1.*L1.*cos(th1))./L1;

end
