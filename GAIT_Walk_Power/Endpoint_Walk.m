function output = Endpoint_Walk(input)

STEP_DIST = input.parameter;
groundFunc = input.auxdata.ground.func;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Objective Function                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
switch input.auxdata.cost.method
    case 'Work'
        output.objective = input.phase(1).integral + input.phase(2).integral;
    case 'CoT'
        Dyn = input.auxdata.dynamics;
        weight = (Dyn.M + 2*Dyn.m)*Dyn.g;
        output.objective = ...
            (input.phase(1).integral + input.phase(2).integral)/weight;
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Discrete Constraints                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

IdxStateSingle = 1:8;
IdxStateDouble = 1:4;

START_D = input.phase(1).initialstate(IdxStateDouble);
END_D = input.phase(1).finalstate(IdxStateDouble);
START_S_Polar = input.phase(2).initialstate(IdxStateSingle);
END_S_Polar = input.phase(2).finalstate(IdxStateSingle);

Duration = input.phase(1).finaltime + input.phase(2).finaltime;

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

%%%% TARGETS %%%%
Swing_Start_x = -STEP_DIST;
Swing_Start_y = feval(groundFunc,-STEP_DIST,[]);
Swing_End_x = STEP_DIST;
Swing_End_y = feval(groundFunc,STEP_DIST,[]);
Hip_Vec_x = 0.5*(Swing_End_x-Swing_Start_x);
Hip_Vec_y = 0.5*(Swing_End_y-Swing_Start_y);


%%%% Hip states continuous at toe-off
output.eventgroup(1).event = END_D - START_S;

%%%% Hip states must move by Hip_Vec over the course of the step
output.eventgroup(2).event = [...
    END_S(1) - START_D(1) - Hip_Vec_x,...
    END_S(2) - START_D(2) - Hip_Vec_y,...
    END_S(3) - START_D(3),...
    END_S(4) - START_D(4)];

%%%% Enfore swing foot initial velocity == 0
output.eventgroup(3).event = [...
    Velocity.footTwo.x(1),...
    Velocity.footTwo.y(1)];

%%%% Enforce step vector for swing foot, horizontal vector
output.eventgroup(4).event = [...
    Position.footTwo.x(1) - Swing_Start_x,...
    Position.footTwo.x(2) - Swing_End_x,...
    Position.footTwo.y(1) - Swing_Start_y,...
    Position.footTwo.y(2) - Swing_End_y,...
    ];

%%%% Speed constratint:
%%%% HACK %%%%    This is only the horizontal component of the speed
output.eventgroup(5).event = ...
    STEP_DIST/Duration;
%%%% DONE %%%%
end