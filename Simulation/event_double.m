function [value,isterminal,direction] = event_double(~,y,P_Event)

% Dummy event - hip crossing ground
value = y(2,:)';   %Hip height
isterminal = false(size(value));
direction = zeros(size(value));

end