function [y, n, c] = ground(x,slope,curvature,step_dist,clearance)

%This function stores the ground profile for computing step vectors and
%swing foot clearance. 
%
% x is the horizontal position of the foot
%
% y is the heigh of the ground at this point
% n is the angle from positive vertical to positive normal
%

%%%% very simple for now
y = slope*x + curvature*x.^2;
dy = slope + 2*curvature*x;
n = atan2(dy,1);

if nargout == 3
   %Then we need the foot clearance as well 
   c = y + clearance*(1-(x./step_dist).^2);
end

end