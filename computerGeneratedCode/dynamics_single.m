function [dStates, contactForces] = dynamics_single(States, Actuators, Parameters)
%function [dStates, contactForces] = dynamics_single(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Dynamics_SingleStance()
% 02-Jan-2014 18:43:48
%
%
% Matthew Kelly 
% Cornell University 
% 

x0 = States(:,1); % (m) Hip horizontal position wrt Foot One
 = States(:,2); % 
 = States(:,3); % 
 = States(:,4); % 
 = States(:,5); % 
 = States(:,6); % 
 = States(:,7); % 
 = States(:,8); % 

F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
Thip = Actuators(:,4); % (Nm) Hip torque applied to Leg Two from Leg One

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity
x2 = Parameters.x2; % (m) Foot Two horizontal position wrt Foot One
y2 = Parameters.y2; % (m) Foot Two vertical position wrt Foot One

dStates = zeros(size(States));
dStates(:,(1:4)) = States(:,(1+4):(4+4));
