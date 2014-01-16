function [Kinematics, Power, Energy] = kinematics_double(States, Actuators, Parameters)
%function [Kinematics, Power, Energy] = kinematics_double(Kinematics, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics_DoubleStance()
% 16-Jan-2014 13:15:10
%
%
% Matthew Kelly 
% Cornell University 
% 

x0 = States(:,1); % (m) Hip horizontal position wrt Foot One
y0 = States(:,2); % (m) Hip vertical position wrt Foot One
dx0 = States(:,3); % (m) Hip horizontal velocity
dy0 = States(:,4); % (m) Hip vertical velocity

m = Parameters.m; % (kg) foot mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity
x2 = Parameters.x2; % (m) Foot Two horizontal position wrt Foot One
y2 = Parameters.y2; % (m) Foot Two vertical position wrt Foot One

L1 = (x0.^2 + y0.^2).^(1./2); %(m) Leg One length
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2); %(m) Leg Two length
dL1 = (dx0.*x0 + dy0.*y0)./L1; %(m/s) Leg One length rate
dL2 = (dx0.*(x0 - x2) + dy0.*(y0 - y2))./L2; %(m/s) Leg Two length rate
th1 = atan2(-x0, y0); %(rad) Leg One angle
th2 = atan2(x2 - x0, y0 - y2); %(rad) Leg Two angle
dth1 = (dy0.*x0 - dx0.*y0)./L1.^2; %(rad/s) Leg One angle rate
dth2 = (dy0.*(x0 - x2) - dx0.*(y0 - y2))./L2.^2; %(rad/s) Leg Two angle rate

Kinematics.L1 = L1;
Kinematics.L2 = L2;
Kinematics.dL1 = dL1;
Kinematics.dL2 = dL2;
Kinematics.th1 = th1;
Kinematics.th2 = th2;
Kinematics.dth1 = dth1;
Kinematics.dth2 = dth2;

if nargout > 1 
    F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
    F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two

    Power.legOne = F1.*dL1;
    Power.legTwo = F2.*dL2;

    Energy.Potential = g.*(M.*y0 + m.*y2);
    Energy.Kinetic = (M.*(dx0.^2 + dy0.^2))./2;
    Energy.Total = Energy.Potential + Energy.Kinetic;
end

end
