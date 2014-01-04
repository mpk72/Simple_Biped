% MAIN_Derive_DoubleStance
%
% This function generates the equations of motion for what I will call the
% "Retractable Double Pendulum" model of walking. It uses the Matlab
% symbolic toolbox to generate the equations of motion, and then
% automatically writes them to a file.
%
% The model consists of a point mass at each foot and at the hip. The feet
% are connected to this hip by an extensible, actuated, leg. Each foot has
% an 'ankle actuator' to provide a control torque, and there is another
% torque actuator at the hip, connecting the two legs.
%
% Written by Matthew Kelly
% December 28, 2013
% Cornell University
%
clc; clear;
addpath ../shared
Directory = '../computerGeneratedCode';  %Write all code in this directory
disp('Running Derive_EoM...')
disp(' -> Defining Model');


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

% The position of the hip  --  STATE
x0 = sym('x0','real'); % Horizontal position of the hip
y0 = sym('y0','real'); % Vertical position of the hip

% The position of foot one  --  KNOWN
x1 = sym(0); % Horizontal position of foot one
y1 = sym(0); % Vertical position of foot one

% The position of foot two  --  KNOWN  --  PARAMETER
x2 = sym('x2','real'); % Horizontal position of foot two
y2 = sym('y2','real'); % Vertical position of foot two

%--  PARAMETER
% Each leg has a small mass at the foot and a large mass at the hip. Since
% the two hip masses are coincident, they are treated as a single mass.
m = sym('m','real');  % Mass of the foot
M = sym('M','real');  % Mass of the hip of the robot
% The system experiences a constant gravitational acceleration:
g = sym('g','real');

%  --  ACTUATOR
% Axial Force along each leg. This force is considered to be an actuator
% input to the system. Compression is positive.
F1 = sym('F1','real'); % Force in the stance leg
F2 = sym('F2','real'); % Force in the swing leg

% No Normal force at the point masses
% No torques in double stance - just axial forces

%  --  CONSTAINT FORCES
% Ground contact forces at each foot
H1 = sym('H1','real'); % Horizontal contact force at foot one
V1 = sym('V1','real'); % Vertical contact force at foot one
H2 = sym('H2','real'); % Horizontal contact force at foot two
V2 = sym('V2','real'); % Vertical contact force at foot two

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        State Derivatives                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

dx0 = sym('dx0','real'); % Horizontal velocity of the hip
dy0 = sym('dy0','real'); % Vertical velocity of the hip
dx1 = sym(0); % Horizontal velocity of foot one
dy1 = sym(0); % Vertical velocity of foot one
dx2 = sym(0); % Horizontal velocity of foot two
dy2 = sym(0); % Vertical velocity of foot two

% The second time derivative of each state. Goal is to find these.
ddx0 = sym('ddx0','real'); % Horizontal acceleration of the hip
ddy0 = sym('ddy0','real'); % Vertical acceleration of the hip
ddx1 = sym(0); % Horizontal acceleration of foot one
ddy1 = sym(0); % Vertical acceleration of foot one
ddx2 = sym(0); % Horizontal acceleration of foot two
ddy2 = sym(0); % Vertical acceleration of foot two



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Coordinate System & Kinematics                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = [1;0;0];  %Positive horizontal axis
j = [0;1;0];  %Positive vertical axis
k = [0;0;1];  %Positive lateral axis

%Relative change in each ordinate along the legs
X1 = (x1-x0); dX1 = (dx1-dx0);
Y1 = (y1-y0); dY1 = (dy1-dy0);
X2 = (x2-x0); dX2 = (dx2-dx0);
Y2 = (y2-y0); dY2 = (dy2-dy0);

%length of both legs:
% This is a commonly used expression that is costly to calculate, so I
% numerically calculate it as an intermediate step.
L1 = sym('L1','real');
L2 = sym('L2','real');
dL1 = sym('dL1','real');
dL2 = sym('dL2','real');

%Angles of both legs, as measured in the k direction from the -j axis:
% th1 = sym('th1','real');
% th2 = sym('th2','real');
dth1 = sym('dth1','real');
dth2 = sym('dth2','real');

% Matlab symbolic toolbox was not being friendly, so I did a small amount
% of the math out by hand. I believe the following to be true:
%
%  L^2 = x^2 + y^2
% (d/dt)(L) = (x*dx + y*dy)/L
% (d/dt)(atan2(x,-y)) = -(dx*y - x*dy)/L^2
%

Kinematics.L1 = sqrt(X1^2 + Y1^2);
Kinematics.L2 = sqrt(X2^2 + Y2^2);

Kinematics.dL1 = (X1*dX1 + Y1*dY1)/L1;
Kinematics.dL2 = (X2*dX2 + Y2*dY2)/L2;

Kinematics.th1 = atan2(X1,-Y1);
Kinematics.th2 = atan2(X2,-Y2);

Kinematics.dth1 = -(dX1*Y1 - X1*dY1)/L1^2;
Kinematics.dth2 = -(dX2*Y2 - X2*dY2)/L2^2;


% Unit vectors pointing from the hip to foot one (a1) and it's normal (b1)
a1 = (X1*i + Y1*j)/L1;
b1 = (-Y1*i + X1*j)/L1;

% Unit vectors pointing from the hip to foot two (a2) and it's normal (b2)
a2 = (X2*i + Y2*j)/L2;
b2 = (-Y2*i + X2*j)/L2;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Position Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Point 0 = Hip
% Point 1 = Foot One
% Point 2 = Foot Two

r0 = x0*i + y0*j;         % Position of Hip(Absolute)
r1 = x1*i + y1*j;         % Position of Foot One (Absolute)
r2 = x2*i + y2*j;         % Position of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Position Derivatives                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

dr0 = dx0*i + dy0*j;         % Velocity of Hip(Absolute)
dr1 = dx1*i + dy1*j;         % Velocity of Foot One (Absolute)
dr2 = dx2*i + dy2*j;         % Velocity of Foot Two (Absolute)

ddr0 = ddx0*i + ddy0*j;         % Acceleration of Hip(Absolute)
ddr1 = ddx1*i + ddy1*j;         % Acceleration of Foot One (Absolute)
ddr2 = ddx2*i + ddy2*j;         % Acceleration of Foot Two (Absolute)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot One                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V1*j + H1*i - m*g*j - F1*a1);
linear_momentum_rate = m*ddr1;

Double_LMB_F1 = simplify(sum_of_forces - linear_momentum_rate);
Double_LMB_F1_i = dot(Double_LMB_F1,i);
Double_LMB_F1_j = dot(Double_LMB_F1,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Hip                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(F1*a1 + F2*a2 - M*g*j);
linear_momentum_rate = M*ddr0;

Double_LMB_H = simplify(sum_of_forces - linear_momentum_rate);
Double_LMB_H_a1 = simplify(dot(Double_LMB_H,a1));
Double_LMB_H_b1 = simplify(dot(Double_LMB_H,b1));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot Two                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(-m*g*j - F2*a2 + H2*i + V2*j);
linear_momentum_rate = m*ddr2;

Double_LMB_F2 = simplify(sum_of_forces - linear_momentum_rate);
Double_LMB_F2_i = dot(Double_LMB_F2,i);
Double_LMB_F2_j = dot(Double_LMB_F2,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%      Collect Implicit Equations of Motion and Phase constraints         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% All phases of motion share these equations. An additional four equations
% are required to solve the system, which come from the 'definition' of
% each phase of motion.

Physics = [...
    Double_LMB_F1_i;
    Double_LMB_F1_j;
    Double_LMB_H_a1;
    Double_LMB_H_b1;
    Double_LMB_F2_i;
    Double_LMB_F2_j;
    ];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Solve Single Stance Dynamics                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Unknowns = [...
    H1;     V1;
    H2;     V2;
    ddx0;    ddy0;
    ];

disp(' -> Solving Single Stance Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dynamics = Soln;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Write Double Stance Dynamics                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing ''dynamics_double.m''')
States = cell(4,2);
States(1,:) = {'x0','(m) Hip horizontal position wrt Foot One'};
States(2,:) = {'y0','(m) Hip vertical position wrt Foot One'};
States(3,:) = {'dx0','(m) Hip horizontal velocity'};
States(4,:) = {'dy0','(m) Hip vertical velocity'};

Contacts = cell(4,2);
Contacts(1,:) = {'H1','(N) Foot One, horizontal contact force'};
Contacts(2,:) = {'V1','(N) Foot One, vertical contact force'};
Contacts(3,:) = {'H2','(N) Foot Two, horizontal contact force'};
Contacts(4,:) = {'V2','(N) Foot Two, vertical contact force'};

Actuators = cell(2,2);
Actuators(1,:) = {'F1','(N) Compresive axial force in Leg One'};
Actuators(2,:) = {'F2','(N) Compresive axial force in Leg Two'};

Parameters = cell(5,2);
Parameters(1,:) = {'m','(kg) foot mass'};
Parameters(2,:) = {'M','(kg) Hip mass'};
Parameters(3,:) = {'g','(m/s^2) Gravity'};
Parameters(4,:) = {'x2','(m) Foot Two horizontal position wrt Foot One'};
Parameters(5,:) = {'y2','(m) Foot Two vertical position wrt Foot One'};

Intermediate = cell(2,2);
Intermediate(1,:) = {'L1', '(m) Leg One length'};
Intermediate(2,:) = {'L2', '(m) Leg Two length'};

input.States = States;
input.Dynamics = Dynamics;
input.Contacts = Contacts;
input.Actuators = Actuators;
input.Parameters = Parameters;
input.Directory = Directory;
input.Intermediate = Intermediate;
input.Kinematics = Kinematics;

Write_Dynamics_DoubleStance(input);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Calculate other useful things                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
Energy.Potential = simplify(M*g*dot(r0,j) + m*g*dot(r2,j) + m*g*dot(r1,j));
Energy.Kinetic = simplify(0.5*M*norm(dr0)^2);

Power.legOne = F1*dL1;
Power.legTwo = F2*dL2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Write Energy, Kinematics, and Power                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Intermediate = cell(8,2);
Intermediate(1,:) = {'L1', '(m) Leg One length'};
Intermediate(2,:) = {'L2', '(m) Leg Two length'};
Intermediate(3,:) = {'dL1', '(m/s) Leg One length rate'};
Intermediate(4,:) = {'dL2', '(m/s) Leg Two length rate'};
Intermediate(5,:) = {'th1', '(rad) Leg One angle'};
Intermediate(6,:) = {'th2', '(rad) Leg Two angle'};
Intermediate(7,:) = {'dth1', '(rad/s) Leg One angle rate'};
Intermediate(8,:) = {'dth2', '(rad/s) Leg Two angle rate'};

input.Intermediate = Intermediate;
input.Kinematics = Kinematics;
input.Energy = Energy;
input.Power = Power;

Write_Kinematics_DoubleStance(input);



