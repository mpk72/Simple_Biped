function [dStates, contactForces] = dynamics(States, Actuators, Parameters, Phase)

%A wrapper function for:
%   dynamics_DoubleStance
%   dynamics_SingleStanceOne
%   dynamics_SingleStanceTwo
%   dynamics_Flight

switch Phase
    case 'D'
        [dStates, contactForces] = dynamics_doubleStance(States, Actuators, Parameters);
    case 'S1'
        [dStates, contactForces] = dynamics_singleStanceOne(States, Actuators, Parameters);
    case 'S2'
        [dStates, contactForces] = dynamics_singleStanceTwo(States, Actuators, Parameters);
    case 'F'
        [dStates, contactForces] = dynamics_flight(States, Actuators, Parameters);
    otherwise
        error('Invalid phase of motion')
        
end

end