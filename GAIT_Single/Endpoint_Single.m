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
Position = kinematics_single(States, Actuators, Parameters);

% % % % % Enfore step length
% % % % output.eventgroup(1).event = [...
% % % %     Position.footTwo.x(1) + L,...
% % % %     Position.footTwo.x(2) - L,...
% % % %     Position.footTwo.y(1),...
% % % %     Position.footTwo.y(2)];

% Enfore step length
output.eventgroup(1).event = [...
    Position.footTwo.y(1),...
    Position.footTwo.y(2)];

end