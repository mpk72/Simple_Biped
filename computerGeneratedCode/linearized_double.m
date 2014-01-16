function [A, B] = linearized_double(States, Actuators, Parameters)
%function [dStates, contactForces] = linearized_double(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Linearized_DoubleStance()
% 16-Jan-2014 12:31:10
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

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity
x2 = Parameters.x2; % (m) Foot Two horizontal position wrt Foot One
y2 = Parameters.y2; % (m) Foot Two vertical position wrt Foot One

Nt = size(States,1);
A = zeros(4,4,Nt);
B = zeros(4,2,Nt);

A(1,3,:) = ones(1,1,Nt);
A(2,4,:) = ones(1,1,Nt);
A(3,1,:) = -(F1.*L2 + F2.*L1)./(L1.*L2.*M);
A(4,2,:) = -(F1.*L2 + F2.*L1)./(L1.*L2.*M);

B(3,1,:) = -x0./(L1.*M);
B(3,2,:) = -(x0 - x2)./(L2.*M);
B(4,1,:) = -y0./(L1.*M);
B(4,2,:) = -(y0 - y2)./(L2.*M);

end
