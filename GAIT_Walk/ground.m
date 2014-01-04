function [y, n] = ground(x,slope)

%This function stores the ground profile for computing step vectors and
%swing foot clearance. 
%
% x is the horizontal position of the foot
%
% y is the heigh of the ground at this point
% n is the angle from positive vertical to positive normal
%

%%%% very simple for now
y = slope*x;
n = atan2(slope,1);

end