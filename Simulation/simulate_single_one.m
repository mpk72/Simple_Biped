function [Data, Final] = simulate_single_one(Initial)
%function [Data, Final] = SIMULATE_SINGLE_ONE(Initial)
%
%FUNCTION:
%   This function runs a simulation in single stance until one of the
%   termination conditions is met.
%
%INPUT:
%   Initial = a struct with Initialization information. Fields:
%       data        = Data.(fieldNames)(end);
%       duration    = Maximum allowable simulation duration
%       dataFreq    = Target sampling frequency for data
%       tolerance   = Passed to reltol and abstol in ode45
%       parameters  = A struct of parameters. Fields:
%           dynamics    - parameters for dynamics
%           control     - parameters for control
%           events      - parameters for events
%
%OUTPUT:
%   Data = a struct with the simulation time-series data. Fields:
%       time    - simulation time stamps
%       x0      - hip position (horizontal)
%       y0      - hip position (vertical)
%       x1      - foot one position (horizontal)
%       y1      - foot one position (vertical)
%       x2      - foot two position (horizontal)
%       y2      - foot two position (vertical)
%       th1     - leg one angle
%       th2     - leg two angle
%       L1      - leg one length
%       L2      - leg two length
%       x0      - hip velocity (horizontal)
%       y0      - hip velocity (vertical)
%       x1      - foot one velocity (horizontal)
%       y1      - foot one velocity (vertical)
%       x2      - foot two velocity (horizontal)
%       y2      - foot two velocity (vertical)
%       dth1    - leg one angle rate
%       dth2    - leg two angle rate
%       dL1     - leg one length rate
%       dL2     - leg two length rate
%       F1      - leg one axial force
%       F2      - leg two axial force
%       T1      - ankle one torque
%       T2      - ankle two torque
%       Thip    - hip torque
%       H1      - foot one horizontal contact force
%       V1      - foot one vertical contact force
%       H2      - foot two horizontal contact force
%       V2      - foot two vertical contact force
%       Power   - a struct with fields:
%           legOne
%           legTwo
%           ankleOne
%           ankleTwo
%           hip
%       Energy  - a struct with fields:
%           Potential
%           Kinetic
%           Total
%
%   Final = a struct with exit information. Fields:
%       nextPhase = {'F','D','S1','S2','FALL'}
%       data = Data.(fieldNames)(end);
%

INITIAL_STATE = [...
    Initial.data.th1;
    Initial.data.th2;
    Initial.data.L1;
    Initial.data.L2;
    Initial.data.dth1;
    Initial.data.dth2;
    Initial.data.dL1;
    Initial.data.dL2;
    ];

TSPAN = Initial.data.time + [0, Initial.duration];

P_Dyn = Initial.parameters.dynamics;
P_Ctl = Initial.parameters.control;
ODEFUN = @(t,y)rhs(t,y,P_Dyn,P_Ctl);

P_Event = Initial.parameters.events;
OPTIONS = odeset(...
    'RelTol', Initial.tolerance,...
    'AbsTol', Initial.tolerance,...
    'Vectorized', 'on',...
    'Events', @(t,y)event_single(t,y,P_Event));

sol = ode45(ODEFUN,TSPAN,INITIAL_STATE,OPTIONS);

%Interpolate data at dataFreq and get time stamps
n = ceil((sol.x(end)-sol.x(1))*Initial.dataFreq);
t = linspace(sol.x(1),sol.x(end),n);
y = deval(sol,t);

%Now format things for analysis:
States = y';
Actuators = control_single(States, P_Ctl);
[~, contactForces] = dynamics_single(States, Actuators, P_Dyn);
[Position, Velocity, Power, Energy] = kinematics_single(States, Actuators, P_Dyn);

Data.time = t';  % simulation time stamps
Data.x0 = Position.hip.x; % hip position
Data.y0 = Position.hip.y;     % hip position
Data.x1 = ones(n,1)*Initial.data.x1;     % foot one position  --  didn't change
Data.y1 = ones(n,1)*Initial.data.y1;     % foot one position  --  didn't change
Data.x2 = Position.footTwo.x;     % foot two position
Data.y2 = Position.footTwo.y;     % foot two position
Data.th1 = States(:,1);    % leg one angle
Data.th2 = States(:,2);     % leg two angle
Data.L1 = States(:,3);      % leg one length
Data.L2 = States(:,4);      % leg two length
Data.dx0 = Velocity.hip.x; % hip velocity
Data.dy0 = Velocity.hip.y;     % hip velocity
Data.dx1 = ones(n,1)*Initial.data.dx1;     % foot one velocity  --  didn't change
Data.dy1 = ones(n,1)*Initial.data.dy1;     % foot one velocity  --  didn't change
Data.dx2 = Velocity.footTwo.x;     % foot two velocity
Data.dy2 = Velocity.footTwo.y;     % foot two velocity
Data.dth1 = States(:,5);    % leg one angle rate
Data.dth2 = States(:,6);    % leg two angle rate
Data.dL1 = States(:,7);     % leg one length rate
Data.dL2 = States(:,8);     % leg two length rate
Data.F1= Actuators(:,1);     % leg one axial force
Data.F2= Actuators(:,2);     % leg two axial force
Data.T1 = Actuators(:,3);    % ankle one torque
Data.T2 = zeros(n,1);     % ankle two torque
Data.Thip = Actuators(:,4);  % hip torque
Data.H1 = contactForces(:,1);     % foot one horizontal contact force
Data.V1 = contactForces(:,2);     % foot one vertical contact force
Data.H2 = zeros(n,1);      % foot two horizontal contact force
Data.V2 = zeros(n,1);      % foot two vertical contact force

names = fieldnames(Data);
for i=1:length(names)
   Final.(names{i})=Data.(names{i})(end);
end

Data.Power = Power;
Data.Energy = Energy;

end

%%%% SUB FUNCTIONS %%%%

function dy = rhs(~, y, P_Dyn, P_Ctl)
States = y';

Actuators = control_single(States, P_Ctl);
dStates = dynamics_single(States, Actuators, P_Dyn);

dy = dStates';
end

%%%%%%%%%%%%% Fields in sol (returned by ode45) %%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %      solver: 'ode45'
% %     extdata: [1x1 struct]
% %           x: [1x31 double]
% %           y: [4x31 double]
% %          xe: 0.3441
% %          ye: [4x1 double]
% %          ie: 1
% %       stats: [1x1 struct]
% %       idata: [1x1 struct]




