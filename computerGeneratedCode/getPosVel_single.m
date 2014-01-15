function [Position, Velocity] = getPosVel_single(States)
%function [Position, Velocity] = getPosVel_single(States)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics_SingleStance()
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

Position.hip.x = -L1.*sin(th1);
Position.hip.y = L1.*cos(th1);
Position.footTwo.x = L2.*sin(th2) - L1.*sin(th1);
Position.footTwo.y = L1.*cos(th1) - L2.*cos(th2);

if nargout==2
    Velocity.hip.x = - dL1.*sin(th1) - L1.*dth1.*cos(th1);
    Velocity.hip.y = dL1.*cos(th1) - L1.*dth1.*sin(th1);
    Velocity.footTwo.x = dL2.*sin(th2) - dL1.*sin(th1) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2);
    Velocity.footTwo.y = dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2);
end

end
