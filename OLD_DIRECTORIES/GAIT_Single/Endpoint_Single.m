function output = Endpoint_Single(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
output.objective = input.phase(1).integral;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Constraints                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
START = input.phase(1).initialstate;
END = input.phase(1).finalstate;
L = input.auxdata.goal.Step_Length;

States = [START;END];
%%%% HACK %%%%
Actuators = zeros(2,4);
%%%% DONE %%%%
Parameters = input.auxdata.dynamics;
[Position, Velocity] = kinematics_single(States, Actuators, Parameters);

% Enfore step length
output.eventgroup(1).event = [...
    Position.footTwo.x(1) + L,...
    Position.footTwo.x(2) - L,...
    Position.footTwo.y(1) + 0,...
    Position.footTwo.y(2) - 0];

% Enfore swing foot initial velocity == 0
output.eventgroup(2).event = [...
    Velocity.footTwo.x(1),...
    Velocity.footTwo.y(1)];

% Make the hip do something reasonable
output.eventgroup(3).event = [...
    Position.hip.x(1) + 0.7*L,...
    Position.hip.x(2) - 0.7*L,...
    ];
output.eventgroup(4).event = [...
    Position.hip.y(1),...
    Position.hip.y(2),...
    ];


end