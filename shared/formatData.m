function Data = formatData(time, phase, States, Actuators, Parameters)



Data.phase = phase;
Data.time = time;
n = length(time);

switch phase
    case 'S1'
        [~, contactForces] = dynamics_single(States, Actuators, Parameters);
        [Position, Velocity, Power, Energy] = kinematics_single(States, Actuators, Parameters);
        
        Data.state.th1 = States(:,1); % (rad) Leg One absolute angle
        Data.state.th2 = States(:,2); % (rad) Leg Two absolute angle
        Data.state.L1 = States(:,3); % (m) Leg One length
        Data.state.L2 = States(:,4); % (m) Leg Two length
        Data.state.dth1 = States(:,5); % (rad/s) Leg One absolute angular rate
        Data.state.dth2 = States(:,6); % (rad/s) Leg Two absolute angular rate
        Data.state.dL1 = States(:,7); % (m/s) Leg One extension rate
        Data.state.dL2 = States(:,8); % (m/s) Leg Two extensioin rate
        Data.state.x1 = zeros(n,1); % (m) Foot One Horizontal Position
        Data.state.y1 = zeros(n,1); % (m) Foot One Vertical Position
        Data.state.x2 = Position.footTwo.x; % (m) Foot Two Horizontal Position
        Data.state.y2 = Position.footTwo.y; % (m) Foot Two Vertical Position
        Data.state.x0 = Position.hip.x; % (m) Hip Horizontal Position
        Data.state.y0 = Position.hip.y; % (m) Hip Vertical Position
        Data.state.dx1 = zeros(n,1); % (m) Foot One Horizontal Velocity
        Data.state.dy1 = zeros(n,1); % (m) Foot One Vertical Velocity
        Data.state.dx2 = Velocity.footTwo.x; % (m) Foot Two Horizontal Velocity
        Data.state.dy2 = Velocity.footTwo.y; % (m) Foot Two Vertical Velocity
        Data.state.dx0 = Velocity.hip.x; % (m) Hip Horizontal Velocity
        Data.state.dy0 = Velocity.hip.y; % (m) Hip Vertical Velocity
        
        Data.control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
        Data.control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
        Data.control.T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
        Data.control.Thip = Actuators(:,4); % (Nm) Hip torque applied to Leg Two from Leg One
        Data.control.T2 = zeros(n,1); % (Nm) External torque applied to Leg Two
        
        Data.contact.H1 = contactForces(:,1); %(N) Foot One, horizontal contact force
        Data.contact.V1 = contactForces(:,2); %(N) Foot One, vertical contact force
        Data.contact.H2 = zeros(n,1); %(N) Foot Two, horizontal contact force
        Data.contact.V2 = zeros(n,1); %(N) Foot Two, vertical contact force
        
        [th1, r1] = cart2pol(Data.contact.H1, Data.contact.V1);
        Data.contact.Mag1 = r1;              %(N) Foot One, contact force magnitude
        Data.contact.Ang1 = th1 - pi/2;      %(rad) Foot One, contact force angle from vertical
        Data.contact.Mag2 = zeros(n,1); %(N) Foot Two, contact force magnitude
        Data.contact.Ang2 = zeros(n,1); %(rad) Foot Two, contact force angle from vertical
        
        Data.power = Power;
        Data.power.ankleTwo = zeros(n,1); %(W) Ankle One, power consumption
        
        Data.energy = Energy;
        
    case 'D'
        [~, contactForces] = dynamics_double(States, Actuators, Parameters);
        [Kinematics, Power, Energy] = kinematics_double(States, Actuators, Parameters);
        
        Data.state.th1 = Kinematics.th1; % (rad) Leg One absolute angle
        Data.state.th2 = Kinematics.th2; % (rad) Leg Two absolute angle
        Data.state.L1 = Kinematics.L1; % (m) Leg One length
        Data.state.L2 = Kinematics.L2; % (m) Leg Two length
        Data.state.dth1 = Kinematics.dth1; % (rad/s) Leg One absolute angular rate
        Data.state.dth2 = Kinematics.dth2; % (rad/s) Leg Two absolute angular rate
        Data.state.dL1 = Kinematics.dL1; % (m/s) Leg One extension rate
        Data.state.dL2 = Kinematics.dL2; % (m/s) Leg Two extensioin rate
        Data.state.x1 = zeros(n,1); % (m) Foot One Horizontal Position
        Data.state.y1 =  zeros(n,1); % (m) Foot One Vertical Position
        Data.state.x2 =  ones(n,1)*Parameters.x2; % (m) Foot Two Horizontal Position
        Data.state.y2 =  ones(n,1)*Parameters.y2; % (m) Foot Two Vertical Position
        Data.state.x0 =  States(:,1); % (m) Hip Horizontal Position
        Data.state.y0 =  States(:,2); % (m) Hip Vertical Position
        Data.state.dx1 = zeros(n,1); % (m) Foot One Horizontal Velocity
        Data.state.dy1 = zeros(n,1); % (m) Foot One Vertical Velocity
        Data.state.dx2 = zeros(n,1); % (m) Foot Two Horizontal Velocity
        Data.state.dy2 = zeros(n,1); % (m) Foot Two Vertical Velocity
        Data.state.dx0 =  States(:,3); % (m) Hip Horizontal Velocity
        Data.state.dy0 =  States(:,4); % (m) Hip Vertical Velocity
        
        Data.control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
        Data.control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
        Data.control.T1 = zeros(n,1); % (Nm) External torque applied to Leg One
        Data.control.Thip = zeros(n,1); % (Nm) Hip torque applied to Leg Two from Leg One
        Data.control.T2 = zeros(n,1); % (Nm) External torque applied to Leg Two
        
        Data.contact.H1 = contactForces(:,1); %(N) Foot One, horizontal contact force
        Data.contact.V1 = contactForces(:,2); %(N) Foot One, vertical contact force
        Data.contact.H2 = contactForces(:,3); %(N) Foot Two, horizontal contact force
        Data.contact.V2 = contactForces(:,4); %(N) Foot Two, vertical contact force
        
        [th1, r1] = cart2pol(Data.contact.H1, Data.contact.V1);
        Data.contact.Mag1 = r1;              %(N) Foot One, contact force magnitude
        Data.contact.Ang1 = th1 - pi/2;      %(rad) Foot One, contact force angle from vertical
        [th2, r2] = cart2pol(Data.contact.H2, Data.contact.V2);
        Data.contact.Mag2 = r2; %(N) Foot Two, contact force magnitude
        Data.contact.Ang2 = th2 - pi/2; %(rad) Foot Two, contact force angle from vertical
        
        Data.power = Power;
        Data.power.ankleOne = zeros(n,1); %(W) Ankle One, power consumption
        Data.power.ankleTwo = zeros(n,1); %(W) Ankle Two, power consumption
        Data.power.hip = zeros(n,1); %(W) Hip, power consumption
        
        Data.energy = Energy;
        
    case 'F'
        [Position, Velocity, Power, Energy] = kinematics_flight(States, Actuators, Parameters);
        
        Data.state.th1 = States(:,3); % (rad) Leg One absolute angle
        Data.state.th2 = States(:,4); % (rad) Leg Two absolute angle
        Data.state.L1 = States(:,5); % (m) Leg One length
        Data.state.L2 = States(:,6); % (m) Leg Two length
        Data.state.dth1 = States(:,9); % (rad/s) Leg One absolute angular rate
        Data.state.dth2 = States(:,10); % (rad/s) Leg Two absolute angular rate
        Data.state.dL1 = States(:,11); % (m/s) Leg One extension rate
        Data.state.dL2 = States(:,12); % (m/s) Leg Two extensioin rate
        Data.state.x1 = States(:,1); % (m) Foot One Horizontal Position
        Data.state.y1 = States(:,2); % (m) Foot One Vertical Position
        Data.state.x2 = Position.footTwo.x; % (m) Foot Two Horizontal Position
        Data.state.y2 = Position.footTwo.y; % (m) Foot Two Vertical Position
        Data.state.x0 = Position.hip.x; % (m) Hip Horizontal Position
        Data.state.y0 = Position.hip.y; % (m) Hip Vertical Position
        Data.state.dx1 = States(:,7); % (m) Foot One Horizontal Velocity
        Data.state.dy1 = States(:,8); % (m) Foot One Vertical Velocity
        Data.state.dx2 = Velocity.footTwo.x; % (m) Foot Two Horizontal Velocity
        Data.state.dy2 = Velocity.footTwo.y; % (m) Foot Two Vertical Velocity
        Data.state.dx0 = Velocity.hip.x; % (m) Hip Horizontal Velocity
        Data.state.dy0 = Velocity.hip.y; % (m) Hip Vertical Velocity
        
        Data.control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
        Data.control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
        Data.control.T1 = zeros(n,1); % (Nm) External torque applied to Leg One
        Data.control.Thip = Actuators(:,3); % (Nm) Hip torque applied to Leg Two from Leg One
        Data.control.T2 = zeros(n,1); % (Nm) External torque applied to Leg Two
        
        Data.contact.H1 = zeros(n,1); %(N) Foot One, horizontal contact force
        Data.contact.V1 = zeros(n,1); %(N) Foot One, vertical contact force
        Data.contact.H2 = zeros(n,1); %(N) Foot Two, horizontal contact force
        Data.contact.V2 = zeros(n,1); %(N) Foot Two, vertical contact force
        
        Data.contact.Mag1 = zeros(n,1);              %(N) Foot One, contact force magnitude
        Data.contact.Ang1 = zeros(n,1);      %(rad) Foot One, contact force angle from vertical
        Data.contact.Mag2 = zeros(n,1); %(N) Foot Two, contact force magnitude
        Data.contact.Ang2 = zeros(n,1); %(rad) Foot Two, contact force angle from vertical
        
        Data.power = Power;
        Data.power.ankleOne = zeros(n,1); %(W) Ankle One, power consumption
        Data.power.ankleTwo = zeros(n,1); %(W) Ankle Two, power consumption
        
        Data.energy = Energy;
        
    otherwise
        error('Unsupported Phase')
end

end