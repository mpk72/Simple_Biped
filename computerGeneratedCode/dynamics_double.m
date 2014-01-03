function [dStates, contactForces] = dynamics_double(States, Actuators, Parameters)
%function [dStates, contactForces] = dynamics_double(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Dynamics_DoubleStance()
% 02-Jan-2014 19:14:09
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

L1 = (x0.^2 + y0.^2).^(1./2); %(m) Leg One length
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2); %(m) Leg Two length

dStates = zeros(size(States));
dStates(:,(1:2)) = States(:,(1+2):(2+2));
dStates(:,3) = -(F1.*L2.*x0 + F2.*L1.*x0 - F2.*L1.*x2)./(L1.*L2.*M);
dStates(:,4) = -(F1.*L2.*y0 + F2.*L1.*y0 - F2.*L1.*y2 + L1.*L2.*M.*g)./(L1.*L2.*M);

% contactForces(:,1) == H1 == (N) Foot One, horizontal contact force
% contactForces(:,2) == V1 == (N) Foot One, vertical contact force
% contactForces(:,3) == H2 == (N) Foot Two, horizontal contact force
% contactForces(:,4) == V2 == (N) Foot Two, vertical contact force
contactForces = zeros(size(States,1),4);
contactForces(:,1) = -(F1.*x0)./L1;
contactForces(:,2) = -(F1.*y0 - L1.*g.*m)./L1;
contactForces(:,3) = -(F2.*(x0 - x2))./L2;
contactForces(:,4) = (F2.*y2 - F2.*y0 + L2.*g.*m)./L2;

end
