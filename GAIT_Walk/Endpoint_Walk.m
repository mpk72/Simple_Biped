function output = Endpoint_Walk(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

output.objective = input.phase(iphase).integral;

START = input.phase(iphase).initialstate;
END = input.phase(iphase).finalstate;
L = input.auxdata.goal.Step_Vector(1);

% Make the hip do something reasonable
IDX_EVENT = 1;
output.eventgroup(IDX_EVENT).event = [...   %horizontal component
    START(1) + 0.6*L,...
    END(1) + 0.1*L,...
    ];
IDX_EVENT = 2;
output.eventgroup(IDX_EVENT).event = [...   %vertical component
    START(2),...
    END(2),...
    ];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

output.objective = input.phase(iphase).integral;

START = input.phase(iphase).initialstate;
END = input.phase(iphase).finalstate;
L = input.auxdata.goal.Step_Vector(1);

States = [START;END];
%%%% HACK %%%%
Actuators = zeros(2,4);
%%%% DONE %%%%
Parameters = input.auxdata.dynamics;
[Position, Velocity] = kinematics_single(States, Actuators, Parameters);

% Enfore step length
IDX_EVENT = 3;
output.eventgroup(IDX_EVENT).event = [...
    Position.footTwo.x(1) + L,...
    Position.footTwo.x(2) - L,...
    Position.footTwo.y(1) + 0,...
    Position.footTwo.y(2) - 0];

% Enfore swing foot initial velocity == 0
IDX_EVENT = 4;
output.eventgroup(IDX_EVENT).event = [...
    Velocity.footTwo.x(1),...
    Velocity.footTwo.y(1)];

% Make the hip do something reasonable
IDX_EVENT = 5;
output.eventgroup(IDX_EVENT).event = [...
    Position.hip.x(1) + 0.7*L,...
    Position.hip.x(2) - 0.7*L,...
    ];
IDX_EVENT = 6;
output.eventgroup(IDX_EVENT).event = [...
    Position.hip.y(1),...
    Position.hip.y(2),...
    ];

end