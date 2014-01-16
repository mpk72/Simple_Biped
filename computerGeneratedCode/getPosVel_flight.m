function [Position, Velocity] = getPosVel_flight(States)
%function [Position, Velocity] = getPosVel_flight(States)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_getPosVel_Flight()
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

Position.hip.x = x - L1.*sin(th1);
Position.hip.y = y + L1.*cos(th1);
Position.footTwo.x = x - L1.*sin(th1) + L2.*sin(th2);
Position.footTwo.y = y + L1.*cos(th1) - L2.*cos(th2);
Position.footOne.x = x;
Position.footOne.y = y;

if nargout==2
    Velocity.hip.x = dx - dL1.*sin(th1) - L1.*dth1.*cos(th1);
    Velocity.hip.y = dy + dL1.*cos(th1) - L1.*dth1.*sin(th1);
    Velocity.footTwo.x = dx - dL1.*sin(th1) + dL2.*sin(th2) - L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2);
    Velocity.footTwo.y = dy + dL1.*cos(th1) - dL2.*cos(th2) - L1.*dth1.*sin(th1) + L2.*dth2.*sin(th2);
    Velocity.footOne.x = dx;
    Velocity.footOne.y = dy;
end

end
