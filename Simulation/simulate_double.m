function [Data, Final] = simulate_double(Initial)
%function [Data, Final] = SIMULATE_DOUBLE(Initial)
%
%FUNCTION:
%   This function runs a simulation in single stance until one of the
%   termination conditions is met.
%
%INPUT:
%   Initial = a struct with Initialization information. Fields:
%       data        = Data.state.(fieldNames)(end);
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
%       phase   - phase code: {'S1','S2','D','F'}
%       time    - simulation time stamps
%       state.x0      - hip position (horizontal)
%       state.y0      - hip position (vertical)
%       state.x1      - foot one position (horizontal)
%       state.y1      - foot one position (vertical)
%       state.x2      - foot two position (horizontal)
%       state.y2      - foot two position (vertical)
%       state.th1     - leg one angle
%       state.th2     - leg two angle
%       state.L1      - leg one length
%       state.L2      - leg two length
%       state.x0      - hip velocity (horizontal)
%       state.y0      - hip velocity (vertical)
%       state.x1      - foot one velocity (horizontal)
%       state.y1      - foot one velocity (vertical)
%       state.x2      - foot two velocity (horizontal)
%       state.y2      - foot two velocity (vertical)
%       state.dth1    - leg one angle rate
%       state.dth2    - leg two angle rate
%       state.dL1     - leg one length rate
%       state.dL2     - leg two length rate
%       control.F1      - leg one axial force
%       control.F2      - leg two axial force
%       control.T1      - ankle one torque
%       control.T2      - ankle two torque
%       control.Thip    - hip torque
%       contact.H1      - foot one horizontal contact force
%       contact.V1      - foot one vertical contact force
%       contact.H2      - foot two horizontal contact force
%       contact.V2      - foot two vertical contact force
%       contact.Mag1    - foot One contact force magnitude
%       contact.Mag2    - foot Two contact force magnitude
%       contact.Ang1    - foot One contact force angle
%       contact.Ang2    - foot Two contact force angle
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
%       nextPhase = {'F','D','S1','S2','FALL','TIMEOUT'}
%       data = Data.(fieldNames)(end);
%

X_shift = Initial.data.x1;
Y_shift = Initial.data.y1;

INITIAL_STATE = [...
    Initial.data.x0 - X_shift;
    Initial.data.y0 - Y_shift;
    Initial.data.dx0;
    Initial.data.dy0;
    ];

TSPAN = Initial.data.time + [0, Initial.duration];

P_Dyn = Initial.parameters.dynamics;
P_Dyn.x2 = Initial.data.x2 - X_shift;
P_Dyn.y2 = Initial.data.y2 - Y_shift;

P_Ctl = Initial.parameters.control;
ODEFUN = @(t,y)rhs(t,y,P_Dyn,P_Ctl);

P_Event = Initial.parameters.events;
OPTIONS = odeset(...
    'RelTol', Initial.tolerance,...
    'AbsTol', Initial.tolerance,...
    'Vectorized', 'on',...
    'Events', @(t,y)event_double(t,y,P_Event));

sol = ode45(ODEFUN,TSPAN,INITIAL_STATE,OPTIONS);

%Interpolate data at dataFreq and get time stamps
n = ceil((sol.x(end)-sol.x(1))*Initial.dataFreq);
t = linspace(sol.x(1),sol.x(end),n);
y = deval(sol,t);

%Now format things for analysis:
States = y';
Actuators = control_double(States, P_Dyn, P_Ctl);
Data = formatData(t', 'D', States, Actuators, P_Dyn);

%Apply translatation so that phases stitch together
E_shift = Y_shift*(2*P_Dyn.m+P_Dyn.M)*P_Dyn.g; %Shift energy datum
Data.state.x1 = Data.state.x1 + X_shift;
Data.state.y1 = Data.state.y1 + Y_shift;   
Data.state.x0 = Data.state.x0 + X_shift;
Data.state.y0 = Data.state.y0 + Y_shift; 
Data.state.x2 = Data.state.x2 + X_shift;
Data.state.y2 = Data.state.y2 + Y_shift; 
Data.energy.Potential = Data.energy.Potential + E_shift;
Data.energy.Total = Data.energy.Total + E_shift;

%Create structure for returns
names = fieldnames(Data.state);
Final.time = Data.time(end);
for i=1:length(names)
   Final.(names{i})=Data.state.(names{i})(end);
end

Final.nextPhase = 'TIMEOUT'; %%%% HACK %%%% replace with events exit

end

%%%% SUB FUNCTIONS %%%%

function dy = rhs(~, y, P_Dyn, P_Ctl)
States = y';

Actuators = control_double(States, P_Dyn, P_Ctl);
dStates = dynamics_double(States, Actuators, P_Dyn);

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




