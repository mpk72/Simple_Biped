% MAIN_Derive_SingleStance
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

% Each leg of the robot has an absolute angle. In both cases this angle is
% measured from the negative vertical axis (-j direction)
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
N1 = sym('N1','real'); % Constraint force orthogonal to leg one
N2 = sym('N2','real'); %Constraint force orthogonal to leg two

% There is a torque motor connecting the two legs, and an ankle motor on
% the stance leg.
T1 = sym('T1','real'); % Ankle Torque, acting on leg one
Thip = sym('Thip','real'); % Hip Torque, acting on leg two (from leg one)

% Ground contact forces at each foot
H1 = sym('H1','real'); % Horizontal contact force at foot one
V1 = sym('V1','real'); % Vertical contact force at foot one

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Coordinate System                                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Inertial reference frame:
i = sym([ 1; 0; 0]);  %Positive horizontal axis
j = sym([ 0; 1; 0]);  %Positive vertical axis
k = sym([ 0; 0; 1]);  %Positive lateral axis

% Frame fixed to leg one:
a1 = cos(th1)*(-j) + sin(th1)*(i);  %Direction from hip to foot one
b1 = -sin(th1)*(-j) + cos(th1)*(i);  %Direction orthogonal to a1

% Frame fixed to leg two:
a2 = cos(th2)*(-j) + sin(th2)*(i);  %Direction from hip to foot two
b2 = -sin(th2)*(-j) + cos(th2)*(i);  %Direction orthogonal to a2


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Position Vectors                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Point 0 = Hip
% Point 1 = Foot One
% Point 2 = Foot Two

r1 = sym([ 0; 0; 0]);

r0 = -L1*a1;         % Position of the Hip (Absolute)

r2 = r0 + L2*a2;      % Position of Foot Two (Absolute)


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

%r1 = sym([ 0; 0; 0]);  %Position of Foot One (Absolute)
dr1 = sym([ 0;0; 0]);
ddr1 = sym([ 0;0; 0]);

%r0 = -L1*a1;         % Position of the Hip (Absolute)
dr0 = -dL1*a1 - L1*da1;
ddr0 = simplify(-ddL1*a1 - 2*dL1*da1 - L1*dda1);

%r2 = r0 + L2*a2;      % Position of Foot Two (Absolute)
dr2 = dr0 + dL2*a2 + L2*da2;
ddr2 = ddr0 + ddL2*a2 + 2*dL2*da2 + L2*dda2;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot One                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(V1*j + H1*i + N1*b1 - m*g*j + F1*a1);
linear_momentum_rate = m*ddr1;

Single_LMB_F1 = simplify(sum_of_forces - linear_momentum_rate);
Single_LMB_F1_i = dot(Single_LMB_F1,i);
Single_LMB_F1_j = dot(Single_LMB_F1,j);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Hip                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(-N1*b1 - F1*a1 - F2*a2 - N2*b2 - M*g*j);
linear_momentum_rate = M*ddr0;

Single_LMB_H = simplify(sum_of_forces - linear_momentum_rate);
Single_LMB_H_a1 = simplify(dot(Single_LMB_H,a1));
Single_LMB_H_b1 = simplify(dot(Single_LMB_H,b1));


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Linear Momentum Balance on Foot Two                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

sum_of_forces = simplify(N2*b2 - m*g*j + F2*a2);
linear_momentum_rate = m*ddr2;

Single_LMB_F2 = simplify(sum_of_forces - linear_momentum_rate);
Single_LMB_F2_i = dot(Single_LMB_F2,i);
Single_LMB_F2_j = dot(Single_LMB_F2,j);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Angular Momentum Balance on Leg One                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
sum_of_moments = simplify(T1*k - Thip*k - L1*N1*k);
angular_momentum_rate = 0*k; %leg is massless

Single_AMB_L1 = simplify(sum_of_moments - angular_momentum_rate);
Single_AMB_L1_k = dot(Single_AMB_L1,k);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Angular Momentum Balance on Leg Two                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
sum_of_moments = simplify(Thip*k - L2*N2*k);
angular_momentum_rate = 0*k; %leg is massless

Single_AMB_L2 = simplify(sum_of_moments - angular_momentum_rate);
Single_AMB_L2_k = dot(Single_AMB_L2,k);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%      Collect Implicit Equations of Motion and Phase constraints         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% All phases of motion share these equations. An additional four equations
% are required to solve the system, which come from the 'definition' of
% each phase of motion.

Physics = [...
    Single_LMB_F1_i;
    Single_LMB_F1_j;
    Single_LMB_H_a1;
    Single_LMB_H_b1;
    Single_LMB_F2_i;
    Single_LMB_F2_j;
    Single_AMB_L1_k;
    Single_AMB_L2_k    ];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Solve Single Stance Dynamics                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Unknowns = [...
    H1;     V1;
    ddth1;  ddth2;
    ddL1;   ddL2;
    N1;     N2];

disp(' -> Solving Single Stance Dynamics')
Soln = jacobSolve(Physics,Unknowns);

Dynamics = Soln;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Write Single Stance Dynamics                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp(' -> Writing ''dynamics_single.m''')
States = cell(8,2);
States(1,:) = {'th1','(rad) Leg One absolute angle'};
States(2,:) = {'th2','(rad) Leg Two absolute angle'};
States(3,:) = {'L1','(m) Leg One length'};
States(4,:) = {'L2','(m) Leg Two length'};
States(5,:) = {'dth1','(rad/s) Leg One absolute angular rate'};
States(6,:) = {'dth2','(rad/s) Leg Two absolute angular rate'};
States(7,:) = {'dL1','(m/s) Leg One extension rate'};
States(8,:) = {'dL2','(m/s) Leg Two extensioin rate'};

Contacts = cell(2,2);
Contacts(1,:) = {'H1','(N) Foot One, horizontal contact force'};
Contacts(2,:) = {'V1','(N) Foot One, vertical contact force'};

Actuators = cell(4,2);
Actuators(1,:) = {'F1','(N) Compresive axial force in Leg One'};
Actuators(2,:) = {'F2','(N) Compresive axial force in Leg Two'};
Actuators(3,:) = {'T1','(Nm) External torque applied to Leg One'};
Actuators(4,:) = {'Thip','(Nm) Hip torque applied to Leg Two from Leg One'};

Parameters = cell(3,2);
Parameters(1,:) = {'m','(kg) foot mass'};
Parameters(2,:) = {'M','(kg) Hip mass'};
Parameters(3,:) = {'g','(m/s^2) Gravity'};

input.States = States;
input.Dynamics = Dynamics;
input.Contacts = Contacts;
input.Actuators = Actuators;
input.Parameters = Parameters;
input.Directory = Directory;

Write_Dynamics_SingleStance(input);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Calculate other useful things                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
Energy.Potential = simplify(m*g*dot(r2,j)) + simplify(M*g*dot(r0,j));
Energy.Kinetic = simplify(0.5*m*norm(dr2).^2) + simplify(0.5*M*norm(dr0)^2);

Position.hip = simplify(r0);
Position.footTwo = simplify(r2);
%Position.footOne = simplify(r1);
Position.CoM = simplify((m*r1 + M*r0 + m*r2)/(2*m + M));

Velocity.hip = simplify(dr0);
Velocity.footTwo = simplify(dr2);
%Velocity.footOne = simplify(dr1);
Velocity.CoM = simplify((m*dr1 + M*dr0 + m*dr2)/(2*m + M));

Power.legOne = F1*dL1;
Power.legTwo = F2*dL2;
Power.ankleOne = T1*dth1;
%Power.ankleTwo = sym('0');
Power.hip = Thip*(dth2-dth1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Write Energy, Kinematics, and Power                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

input.Energy = Energy;
input.Position = Position;
input.Velocity = Velocity;
input.Power = Power;

Write_Kinematics_SingleStance(input);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Compute Linearized Equations                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[A, B] = getLinearizedEqns(States,Actuators,Dynamics);

input.A = A;
input.B = B;

Write_Linearized_SingleStance(input);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Write getPower()                                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%This is optimized for runtime in a cost function

Write_getPower_SingleStance(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Write getPosVel()                                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%This is optimized for use in optimization

Write_getPosVel_SingleStance(input)




