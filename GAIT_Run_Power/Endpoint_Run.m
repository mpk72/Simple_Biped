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

START = 1; END = 2;
FLIGHT = 1; SINGLE = 2;

States_Flight = [input.phase(FLIGHT).initialstate(:,1:12);...
    input.phase(FLIGHT).finalstate(:,1:12)];
[Pos_F, Vel_F] = getPosVel_flight(States_Flight);

States_Single = [input.phase(SINGLE).initialstate(:,1:8);...
    input.phase(SINGLE).finalstate(:,1:8)];
[Pos_S, Vel_S] = getPosVel_single(States_Single);

Step_Vec = zeros(1,2);   
Step_Vec(1) = STEP_DIST;   %Horizontal Component
Step_Vec(2) = -feval(groundFunc,-STEP_DIST,[]); %Foot Two @ Start of Flight

%%%% Heel-Strike Defect in Hip
output.eventgroup(1).event = [...
    Pos_S.hip.x(START) - Pos_F.hip.x(END),...
    Pos_S.hip.y(START) - Pos_F.hip.y(END),...
    Vel_S.hip.x(START) - Vel_F.hip.x(END),...
    Vel_S.hip.y(START) - Vel_F.hip.y(END)];

%%%% Heel-Strike Defect in Foot Two
output.eventgroup(2).event = [...
    Pos_S.footTwo.x(START) - Pos_F.footTwo.x(END),...
    Pos_S.footTwo.y(START) - Pos_F.footTwo.y(END),...
    Vel_S.footTwo.x(START) - Vel_F.footTwo.x(END),...
    Vel_S.footTwo.y(START) - Vel_F.footTwo.y(END)];

%%%% Heel-Strike Defect in Foot One  (Impact Location at origin)
output.eventgroup(3).event = [...
    Pos_F.footOne.x(END),...
    Pos_F.footOne.y(END)];

%%%% Foot Two Initial State (Step vector and periodic constraint)
output.eventgroup(4).event = [...
    Pos_F.footTwo.x(START) + Step_Vec(1),...
    Pos_F.footTwo.y(START) + Step_Vec(2),...
    Vel_F.footTwo.x(START),...
    Vel_F.footTwo.y(START)];

%%%% Hip Translation (periodic constraint)
output.eventgroup(5).event = [...
    Pos_S.hip.x(END) - Pos_F.hip.x(START),...
    Pos_S.hip.y(END) - Pos_F.hip.y(START),...
    Vel_S.hip.x(END) - Vel_F.hip.x(START),...
    Vel_S.hip.y(END) - Vel_F.hip.y(START)];

end

