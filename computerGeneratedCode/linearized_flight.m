function [A, B] = linearized_flight(States, Actuators, Parameters)
%function [dStates, contactForces] = linearized_flight(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Linearized_Flight()
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

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

Nt = size(States,1);
A = zeros(12,12,Nt);
B = zeros(12,3,Nt);

A(1,7,:) = ones(1,1,Nt);
A(2,8,:) = ones(1,1,Nt);
A(3,9,:) = ones(1,1,Nt);
A(4,10,:) = ones(1,1,Nt);
A(5,11,:) = ones(1,1,Nt);
A(6,12,:) = ones(1,1,Nt);
A(7,3,:) = (Thip.*sin(th1) + F1.*L1.*cos(th1))./(L1.*m);
A(7,5,:) = (Thip.*cos(th1))./(L1.^2.*m);
A(8,3,:) = -(Thip.*cos(th1) - F1.*L1.*sin(th1))./(L1.*m);
A(8,5,:) = (Thip.*sin(th1))./(L1.^2.*m);
A(9,3,:) = -(Thip.*sin(th1 - th2) + F2.*L2.*cos(th1 - th2))./(L1.*L2.*M);
A(9,4,:) = (Thip.*sin(th1 - th2) + F2.*L2.*cos(th1 - th2))./(L1.*L2.*M);
A(9,5,:) = (2.*L2.*M.*Thip + 2.*L2.*Thip.*m - L1.*Thip.*m.*cos(th1 - th2) + F2.*L1.*L2.*m.*sin(th1 - th2) + 2.*L1.*L2.*M.*dL1.*dth1.*m)./(L1.^3.*L2.*M.*m);
A(9,6,:) = -(Thip.*cos(th1 - th2))./(L1.*L2.^2.*M);
A(9,9,:) = -(2.*dL1)./L1;
A(9,11,:) = -(2.*dth1)./L1;
A(10,3,:) = (Thip.*sin(th1 - th2) + F1.*L1.*cos(th1 - th2))./(L1.*L2.*M);
A(10,4,:) = -(Thip.*sin(th1 - th2) + F1.*L1.*cos(th1 - th2))./(L1.*L2.*M);
A(10,5,:) = (Thip.*cos(th1 - th2))./(L1.^2.*L2.*M);
A(10,6,:) = -(2.*L1.*M.*Thip + 2.*L1.*Thip.*m - L2.*Thip.*m.*cos(th1 - th2) + F1.*L1.*L2.*m.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m)./(L1.*L2.^3.*M.*m);
A(10,10,:) = -(2.*dL2)./L2;
A(10,12,:) = -(2.*dth2)./L2;
A(11,3,:) = (Thip.*cos(th1 - th2) - F2.*L2.*sin(th1 - th2))./(L2.*M);
A(11,4,:) = -(Thip.*cos(th1 - th2) - F2.*L2.*sin(th1 - th2))./(L2.*M);
A(11,5,:) = dth1.^2;
A(11,6,:) = -(Thip.*sin(th1 - th2))./(L2.^2.*M);
A(11,9,:) = 2.*L1.*dth1;
A(12,3,:) = (Thip.*cos(th1 - th2) - F1.*L1.*sin(th1 - th2))./(L1.*M);
A(12,4,:) = -(Thip.*cos(th1 - th2) - F1.*L1.*sin(th1 - th2))./(L1.*M);
A(12,5,:) = -(Thip.*sin(th1 - th2))./(L1.^2.*M);
A(12,6,:) = dth2.^2;
A(12,10,:) = 2.*L2.*dth2;

B(7,1,:) = sin(th1)./m;
B(7,3,:) = -cos(th1)./(L1.*m);
B(8,1,:) = -cos(th1)./m;
B(8,3,:) = -sin(th1)./(L1.*m);
B(9,2,:) = -sin(th1 - th2)./(L1.*M);
B(9,3,:) = -(L2.*m + L2.*M - L1.*m.*cos(th1 - th2))./(L1.^2.*L2.*M.*m);
B(10,1,:) = sin(th1 - th2)./(L2.*M);
B(10,3,:) = (L1.*m + L1.*M - L2.*m.*cos(th1 - th2))./(L1.*L2.^2.*M.*m);
B(11,1,:) = (M + m)./(M.*m);
B(11,2,:) = cos(th1 - th2)./M;
B(11,3,:) = sin(th1 - th2)./(L2.*M);
B(12,1,:) = cos(th1 - th2)./M;
B(12,2,:) = (M + m)./(M.*m);
B(12,3,:) = sin(th1 - th2)./(L1.*M);

end
