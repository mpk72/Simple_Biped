function output = Endpoint_Run(input)

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

IdxStateFlight = 1:12;
IdxStateSingle = 1:8;

START_F_Polar = input.phase(1).initialstate(IdxStateFlight);
END_F_Polar = input.phase(1).finalstate(IdxStateFlight);
START_S_Polar = input.phase(2).initialstate(IdxStateSingle);
END_S_Polar = input.phase(2).finalstate(IdxStateSingle);

Duration = input.phase(1).finaltime + input.phase(2).finaltime;

[Pos_F, Vel_F] = getPosVel_flight([START_F_Polar;END_F_Polar]);
[Pos_S, Vel_S] = getPosVel_single([START_S_Polar;END_S_Polar]);

%Thes happen in this order
START_F = [...    %Start of flight phase
    Pos_F.hip.x(1),...
    Pos_F.hip.y(1),...
    Vel_F.hip.x(1),...
    Vel_F.hip.y(1)];
END_F = [...
    Pos_F.hip.x(2),...
    Pos_F.hip.y(2),...
    Vel_F.hip.x(2),...
    Vel_F.hip.y(2)];
START_S = [...
    Pos_S.hip.x(1),...
    Pos_S.hip.y(1),...
    Vel_S.hip.x(1),...
    Vel_S.hip.y(1)];
END_S = [...       %End of swing phase
    Pos_S.hip.x(2),...
    Pos_S.hip.y(2),...
    Vel_S.hip.x(2),...
    Vel_S.hip.y(2)];

%%%% ABOUT %%%%
% In single stance, Foot One is the stance leg. For things to line up
% nicely we want the Foot One in flight to become the stance leg. Thus,
% Foot Two will be the trailing leg during flight. 

%%%% TARGETS %%%%
Foot_Two_Start_x = -STEP_DIST;
Foot_Two_Start_y = feval(groundFunc,-STEP_DIST,[]);
Foot_Two_End_x = STEP_DIST;
Foot_Two_End_y = feval(groundFunc,STEP_DIST,[]);
Step_Vec_x = 0.5*(Foot_Two_End_x-Foot_Two_Start_x);
Step_Vec_y = 0.5*(Foot_Two_End_y-Foot_Two_Start_y);

%%%% Hip states continuous at impact (Flight -> Single Stance)
output.eventgroup(1).event = END_F - START_S;

%%%% Hip states must move by Hip_Vec over the course of the step
output.eventgroup(2).event = [...
    END_S(1) - START_F(1) - Step_Vec_x,...
    END_S(2) - START_F(2) - Step_Vec_y,...
    END_S(3) - START_F(3),...
    END_S(4) - START_F(4)];

%%%% Foot Two state must be continuous at heel strike
output.eventgroup(3).event = [...
    Pos_S.footTwo.x(1) - Pos_F.footTwo.x(2),...
    Pos_S.footTwo.y(1) - Pos_F.footTwo.y(2),...
    Vel_S.footTwo.x(1) - Vel_F.footTwo.x(2),...
    Vel_S.footTwo.y(1) - Vel_F.footTwo.y(2),...
    ];

%%%% Enforce step vector for swing foot, horizontal vector
output.eventgroup(4).event = [...
    Pos_S.footTwo.x(2) - Pos_F.footTwo.x(1) - Step_Vec_x,...
    Pos_S.footTwo.y(2) - Pos_F.footTwo.y(1) - Step_Vec_y,...
    Vel_S.footTwo.x(2) - Vel_F.footTwo.x(1),...
    Vel_S.footTwo.y(2) - Vel_F.footTwo.y(1),...
    ];

%%%% Foot One impacts ground at origin
output.eventgroup(5).event = [...
    Pos_F.footTwo.x(2),...
    Pos_F.footTwo.y(2),...
];

%%%% Speed constratint:
%%%% HACK %%%%    This is only the horizontal component of the speed
output.eventgroup(6).event = ...
    STEP_DIST/Duration;
%%%% DONE %%%%
end