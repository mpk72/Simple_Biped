function [value,isterminal,direction] = event_single(~,y,P_Event)

Position = getPosVel_single(y');

% Dummy event - crossing ground
value = Position.footTwo.y;
isterminal = false(size(value));
direction = zeros(size(value));

end