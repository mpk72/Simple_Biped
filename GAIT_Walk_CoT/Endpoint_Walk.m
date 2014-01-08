function output = Endpoint_Walk(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Objective Function                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
output.objective = input.phase(1).integral + input.phase(2).integral;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Discrete Constraints                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

IdxStateDouble = 1:4;
IdxStateSingle = 1:8;

START_D = input.phase(1).initialstate(IdxStateDouble);
END_D = input.phase(1).finalstate(IdxStateDouble);
START_S_Polar = input.phase(2).initialstate(IdxStateSingle);
END_S_Polar = input.phase(2).finalstate(IdxStateSingle);

States = [START_S_Polar;END_S_Polar];
[Position, Velocity] = getPosVel_single(States);

START_S = [...
    Position.hip.x(1),...
    Position.hip.y(1),...
    Velocity.hip.x(1),...
    Velocity.hip.y(1)];
END_S = [...
    Position.hip.x(2),...
    Position.hip.y(2),...
    Velocity.hip.x(2),...
    Velocity.hip.y(2)];

%%%% Hip states continuous at toe-off
output.eventgroup(1).event = END_D - START_S;

%%%% Hip states must move by STEP_VECTOR over the course of the step
output.eventgroup(2).event = [...
    END_S(1) - START_D(1),...
    END_S(2) - START_D(2),...
    END_S(3) - START_D(3),...
    END_S(4) - START_D(4)];

%%%% Enfore swing foot initial velocity == 0
output.eventgroup(3).event = [...
    Velocity.footTwo.x(1),...
    Velocity.footTwo.y(1)];

%%%% Enforce step vector for swing foot, horizontal vector
output.eventgroup(4).event = [...
    Position.footTwo.x(1),...
    Position.footTwo.x(2),...
    Position.footTwo.y(1),...
    Position.footTwo.y(2),...
    ];

end