function [Position, Velocity, Power, Energy] = kinematics_flight(States, Actuators, Parameters)
%function [Position, Velocity, Power, Energy] = kinematics_flight(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics_Flight()
% 16-Jan-2014 13:16:35
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

Position.hip.x = x - L1.*sin(th1);
Position.hip.y = y + L1.*cos(th1);
Position.footTwo.x = x - L1.*sin(th1) + L2.*sin(th2);
Position.footTwo.y = y + L1.*cos(th1) - L2.*cos(th2);
Position.footOne.x = x;
Position.footOne.y = y;
Position.CoM.x = (M.*(x - L1.*sin(th1)) + m.*x + m.*(x - L1.*sin(th1) + L2.*sin(th2)))./(M + 2.*m);
Position.CoM.y = (M.*(y + L1.*cos(th1)) + m.*y + m.*(y + L1.*cos(th1) - L2.*cos(th2)))./(M + 2.*m);

Velocity.hip.x = dx - dL1.*sin(th1) - L1.*dth1.*cos(th1);
Velocity.hip.y = dy + dL1.*cos(th1) - L1.*dth1.*sin(th1);
Velocity.footTwo.x = dx - dL1.*sin(th1) + dL2.*sin(th2) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2);
Velocity.footTwo.y = dy + dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2);
Velocity.footOne.x = dx;
Velocity.footOne.y = dy;
Velocity.CoM.x = (dx.*m - M.*(dL1.*sin(th1) - dx + L1.*dth1.*cos(th1)) + m.*(dx - dL1.*sin(th1) + dL2.*sin(th2) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2)))./(M + 2.*m);
Velocity.CoM.y = (M.*(dy + dL1.*cos(th1) - L1.*dth1.*sin(th1)) + dy.*m + m.*(dy + dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2)))./(M + 2.*m);

Power.legOne = F1.*dL1;
Power.legTwo = F2.*dL2;
Power.hip = -Thip.*(dth1 - dth2);

Energy.Potential = g.*m.*(y + L1.*cos(th1) - L2.*cos(th2)) + M.*g.*(y + L1.*cos(th1)) + g.*m.*y;
Energy.Kinetic = (m.*(dx.^2 + dy.^2))./2 + (m.*((dx - dL1.*sin(th1) + dL2.*sin(th2) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2)).^2 + (dy + dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2)).^2))./2 + (M.*((dL1.*sin(th1) - dx + L1.*dth1.*cos(th1)).^2 + (dy + dL1.*cos(th1) - L1.*dth1.*sin(th1)).^2))./2;
Energy.Total = Energy.Potential + Energy.Kinetic;

end
