%NAMESPACE_DYNAMICS

%This script is shared by all functions that derive equations of motion for
%the model. This ensures that there are common naming conventions for all
%phases of motion, and that all of the kinematics are calcualted the same
%exact way for every function.

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Model                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% This model of walking is comprised of three point masses: one at each
% foot and one at the hip. The equations are derived for three phases of
% motion: Flight, Single Stance, and Double Stance. The lets in this
% walking model are massless and have variable length. There are five
% controls available to the system: force actuator in each leg, foot torque
% when foot is in contact with the ground, and hip torque. There are six
% degrees of freedom in this model: position of foot one (x,y), absolute
% angle of each leg (th1,th2), and length of each leg (L1, L2). 
%
% Some states and actuators are not used in some of the phases. More detail
% to follow, in the description for each section.

% Each leg of the robot has an absolute angle. In both cases this angle is
% measured from the negative vertical axis (i direction)
th1 = sym('th1','real');    % Absolute angle of leg 1
th2 = sym('th2','real');  % Absolute angle of leg 2

% Each leg of the robot has a variable length:
L1 = sym('L1','real'); % Length of leg one, must be positive
L2 = sym('L2','real'); % Length of leg two, must be positive

% Each leg has a small mass at the foot and a large mass at the hip. Since
% the two hip masses are coincident, they are treated as a single mass. 
m = sym('m','real');  % Mass of the foot
M = sym('M','real');  % Mass of the hip of the robot 

% The system experiences a constant gravitational acceleration:
g = sym('g','real');

% Axial Force along each leg. This force is considered to be an actuator 
% input to the system. Compression is positive.
F1 = sym('F1','real'); % Force in the stance leg
F2 = sym('F2','real'); % Force in the swing leg

% Constraint force at each joint. This force is always orthogonal to Fi
N1 = sym('N1','real'); % Constraint force at foot one
N2 = sym('N2','real'); % constraint force between legs

% There is a torque motor connecting the two legs, and an ankle motor on
% the stance leg.
T1 = sym('T1','real'); % Ankle Torque, acting on leg one
T2 = sym('T2','real'); % Hip Torque, acting on leg two (from leg one)

% Ground contact forces at each foot
H1 = sym('H1','real'); % Horizontal contact force at foot one
V1 = sym('V1','real'); % Vertical contact force at foot one
H2 = sym('H2','real'); % Horizontal contact force at foot two
V2 = sym('V2','real'); % Vertical contact force at foot two


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Coordinate System                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = sym([ 0;-1; 0]);  %Positive horizontal axis
j = sym([ 1; 0; 0]);  %Positive vertical axis
k = sym([ 0; 0; 1]);  %Positive lateral axis

% Frame fixed to leg one:
a1 = cos(th1)*i + sin(th1)*j;  %Direction from foot one to hip
b1 = -sin(th1)*i + cos(th1)*j;  %Direction orthogonal to a1   

% Frame fixed to leg two:
a2 = cos(th2)*i + sin(th2)*j;  %Direction from hip to foot two
b2 = -sin(th2)*i + cos(th2)*j;  %Direction orthogonal to a2   


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Position Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Point 0 = Foot One
% Point 1 = Hip
% Point 2 = Foot Two

r1 = L1*a1;            % Position of the Hip (Absolute)

r21 = L2*a2;            % Position of foot two with respect to the hip:
r2 = r1 + r21;    % Position of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        State Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% The first time derivative of each state. These are considered known.
dth1 = sym('dth1','real');  
dth2 = sym('dth2','real'); 
dL1 = sym('dL1','real');
dL2 = sym('dL2','real');

% The second time derivative of each state. Goal is to find these.
ddth1 = sym('ddth1','real');  
ddth2 = sym('ddth2','real'); 
ddL1 = sym('ddL1','real');
ddL2 = sym('ddL2','real');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Frame Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%First time derivative of the stance frame
da1 = dth1*b1;
db1 = -dth1*a1;

%First time derivative of the swing frame
da2 = dth2*b2;
db2 = -dth2*a2;

%Second time derivative of the stance frame:
dda1 = simplify(ddth1*b1 - dth1^2*a1);
ddb1 = simplify(-ddth1*a1 - dth1^2*b1);

%Second time derivative of the swing frame:
dda2 = simplify(ddth2*b2 - dth2^2*a2);
ddb2 = simplify(-ddth2*a2 - dth2^2*b2);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Position Derivatives                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


dr1 = dL1*a1 + L1*da1;     % Velocity of hip with respect to the origin

dr21 = dL2*a2 + L2*da2;     % Velocity of foot two with respect to the hip:
dr2 = dr1 + dr21;    % Velocity of Foot Two (Absolute)


  % Acc. of hip wrt the foot one:
ddr1 = simplify(ddL1*a1 + 2*dL1*da1 + L1*dda1);

ddr21 = ddL2*a2 + 2*dL2*da2 + L2*dda2;     % Acc. of foot two wrt the hip:
ddr21 = simplify(ddr21);
ddr2 = ddr1 + ddr21;               % Acc. of Foot Two (Absolute)


