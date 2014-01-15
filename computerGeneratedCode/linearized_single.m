function [A, B] = linearized_single(States, Actuators, Parameters)
%function [dStates, contactForces] = linearized_single(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Linearized_SingleStance()
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

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

Nt = size(States,1);
A = zeros(8,8,Nt);
B = zeros(8,4,Nt);

A(1,5,:) = ones(1,1,Nt);
A(2,6,:) = ones(1,1,Nt);
A(3,7,:) = ones(1,1,Nt);
A(4,8,:) = ones(1,1,Nt);
A(5,1,:) = -(Thip.*sin(th1 - th2) + F2.*L2.*cos(th1 - th2) - L2.*M.*g.*cos(th1))./(L1.*L2.*M);
A(5,2,:) = (Thip.*sin(th1 - th2) + F2.*L2.*cos(th1 - th2))./(L1.*L2.*M);
A(5,3,:) = -(2.*L2.*T1 - 2.*L2.*Thip + L1.*Thip.*cos(th1 - th2) - F2.*L1.*L2.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL1.*dth1 + L1.*L2.*M.*g.*sin(th1))./(L1.^3.*L2.*M);
A(5,4,:) = -(Thip.*cos(th1 - th2))./(L1.*L2.^2.*M);
A(5,5,:) = -(2.*dL1)./L1;
A(5,7,:) = -(2.*dth1)./L1;
A(6,1,:) = (Thip.*sin(th1 - th2) - T1.*sin(th1 - th2) + F1.*L1.*cos(th1 - th2))./(L1.*L2.*M);
A(6,2,:) = -(Thip.*sin(th1 - th2) - T1.*sin(th1 - th2) + F1.*L1.*cos(th1 - th2))./(L1.*L2.*M);
A(6,3,:) = -(cos(th1 - th2).*(T1 - Thip))./(L1.^2.*L2.*M);
A(6,4,:) = -(2.*L1.*M.*Thip + 2.*L1.*Thip.*m + L2.*T1.*m.*cos(th1 - th2) - L2.*Thip.*m.*cos(th1 - th2) + F1.*L1.*L2.*m.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m)./(L1.*L2.^3.*M.*m);
A(6,6,:) = -(2.*dL2)./L2;
A(6,8,:) = -(2.*dth2)./L2;
A(7,1,:) = (Thip.*cos(th1 - th2) - F2.*L2.*sin(th1 - th2) + L2.*M.*g.*sin(th1))./(L2.*M);
A(7,2,:) = -(Thip.*cos(th1 - th2) - F2.*L2.*sin(th1 - th2))./(L2.*M);
A(7,3,:) = dth1.^2;
A(7,4,:) = -(Thip.*sin(th1 - th2))./(L2.^2.*M);
A(7,5,:) = 2.*L1.*dth1;
A(8,1,:) = -(T1.*cos(th1 - th2) - Thip.*cos(th1 - th2) + F1.*L1.*sin(th1 - th2))./(L1.*M);
A(8,2,:) = (T1.*cos(th1 - th2) - Thip.*cos(th1 - th2) + F1.*L1.*sin(th1 - th2))./(L1.*M);
A(8,3,:) = (sin(th1 - th2).*(T1 - Thip))./(L1.^2.*M);
A(8,4,:) = dth2.^2;
A(8,6,:) = 2.*L2.*dth2;

B(5,2,:) = -sin(th1 - th2)./(L1.*M);
B(5,3,:) = 1./(L1.^2.*M);
B(5,4,:) = -(L2 - L1.*cos(th1 - th2))./(L1.^2.*L2.*M);
B(6,1,:) = sin(th1 - th2)./(L2.*M);
B(6,3,:) = cos(th1 - th2)./(L1.*L2.*M);
B(6,4,:) = (L1.*m + L1.*M - L2.*m.*cos(th1 - th2))./(L1.*L2.^2.*M.*m);
B(7,1,:) = 1./M;
B(7,2,:) = cos(th1 - th2)./M;
B(7,4,:) = sin(th1 - th2)./(L2.*M);
B(8,1,:) = cos(th1 - th2)./M;
B(8,2,:) = (M + m)./(M.*m);
B(8,3,:) = -sin(th1 - th2)./(L1.*M);
B(8,4,:) = sin(th1 - th2)./(L1.*M);

end
