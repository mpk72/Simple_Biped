function [Position, Velocity, Power, Energy] = kinematics_single(States, Actuators, Parameters)
%function [Position, Velocity, Power, Energy] = kinematics_single(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics_SingleStance()
% 15-Jan-2014 13:47:05
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

Position.hip.x = -L1.*sin(th1);
Position.hip.y = L1.*cos(th1);
Position.footTwo.x = L2.*sin(th2) - L1.*sin(th1);
Position.footTwo.y = L1.*cos(th1) - L2.*cos(th2);
Position.CoM.x = -(m.*(L1.*sin(th1) - L2.*sin(th2)) + L1.*M.*sin(th1))./(M + 2.*m);
Position.CoM.y = (m.*(L1.*cos(th1) - L2.*cos(th2)) + L1.*M.*cos(th1))./(M + 2.*m);

Velocity.hip.x = - dL1.*sin(th1) - L1.*dth1.*cos(th1);
Velocity.hip.y = dL1.*cos(th1) - L1.*dth1.*sin(th1);
Velocity.footTwo.x = dL2.*sin(th2) - dL1.*sin(th1) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2);
Velocity.footTwo.y = dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2);
Velocity.CoM.x = -(m.*(dL1.*sin(th1) - dL2.*sin(th2) + L1.*dth1.*cos(th1) - L2.*dth2.*cos(th2)) + M.*(dL1.*sin(th1) + L1.*dth1.*cos(th1)))./(M + 2.*m);
Velocity.CoM.y = (m.*(dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2)) + M.*(dL1.*cos(th1) - L1.*dth1.*sin(th1)))./(M + 2.*m);

Power.legOne = F1.*dL1;
Power.legTwo = F2.*dL2;
Power.ankleOne = T1.*dth1;
Power.hip = -Thip.*(dth1 - dth2);

Energy.Potential = g.*m.*(L1.*cos(th1) - L2.*cos(th2)) + L1.*M.*g.*cos(th1);
Energy.Kinetic = (M.*(dL1.^2 + L1.^2.*dth1.^2))./2 + (m.*((dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2)).^2 + (dL1.*sin(th1) - dL2.*sin(th2) + L1.*dth1.*cos(th1) - L2.*dth2.*cos(th2)).^2))./2;
Energy.Total = Energy.Potential + Energy.Kinetic;

end
