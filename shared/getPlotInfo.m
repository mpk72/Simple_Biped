function plotInfo = getPlotInfo(output)

P = output.result.setup.auxdata;
nPhase = length(P.phase);
D = zeros(nPhase,0);

for iphase = 1:length(P.phase)
    
    D(iphase).time = output.result.interpsolution.phase(iphase).time;
    D(iphase).phase = P.phase{iphase};
    nTime = length(D(iphase).time);
    
    switch D(iphase).phase
        case 'S1' %Single stance, with Stance_Foot == Foot_One
            
            States = output.result.interpsolution.phase(iphase).state;
            D(iphase).state.th1 = States(:,1); % (rad) Leg One absolute angle
            D(iphase).state.th2 = States(:,2); % (rad) Leg Two absolute angle
            D(iphase).state.L1 = States(:,3); % (m) Leg One length
            D(iphase).state.L2 = States(:,4); % (m) Leg Two length
            D(iphase).state.dth1 = States(:,5); % (rad/s) Leg One absolute angular rate
            D(iphase).state.dth2 = States(:,6); % (rad/s) Leg Two absolute angular rate
            D(iphase).state.dL1 = States(:,7); % (m/s) Leg One extension rate
            D(iphase).state.dL2 = States(:,8); % (m/s) Leg Two extensioin rate
            D(iphase).state.x = zeros(nTime,1); % (m) Foot One Horizontal Position
            D(iphase).state.y = zeros(nTime,1); % (m) Foot One Vertical Position
            D(iphase).state.dx = zeros(nTime,1); % (m) Foot One Horizontal Velocity
            D(iphase).state.dy = zeros(nTime,1); % (m) Foot One Vertical Velocity
            
            Actuators = output.result.interpsolution.phase(iphase).control;
            D(iphase).control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
            D(iphase).control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
            D(iphase).control.T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
            D(iphase).control.Thip = Actuators(:,4); % (Nm) Hip torque applied to Leg Two from Leg One
            D(iphase).control.T2 = zeros(nTime,1); % (Nm) External torque applied to Leg Two
            
            Parameters = P.dynamics;
            [~, contactForces] = dynamics_single(States, Actuators, Parameters);
            
            D(iphase).contact.H1 = contactForces(:,1); %(N) Foot One, horizontal contact force
            D(iphase).contact.V1 = contactForces(:,2); %(N) Foot One, vertical contact force
            D(iphase).contact.H2 = zeros(nTime,1); %(N) Foot Two, horizontal contact force
            D(iphase).contact.V2 = zeros(nTime,1); %(N) Foot Two, vertical contact force
            
            [th, r] = cart2pol(D(iphase).contact.H1, D(iphase).contact.V1);
            D(iphase).contact.Mag1 = r;              %(N) Foot One, contact force magnitude
            D(iphase).contact.Ang1 = th - pi/2;      %(rad) Foot One, contact force angle from vertical
            D(iphase).contact.Mag2 = zeros(nTime,1); %(N) Foot Two, contact force magnitude
            D(iphase).contact.Ang2 = zeros(nTime,1); %(rad) Foot Two, contact force angle from vertical
            D(iphase).contact.Bnd1 = P.ground.normal.bounds + P.ground.normal.single.one; %Bounds on the contact angle
            D(iphase).contact.Bnd2 = zeros(2,1);
            
            [Position, Velocity, Power, Energy] = kinematics_single(States, Actuators, Parameters);
            
            D(iphase).position = Position;
            D(iphase).position.footOne.x = zeros(nTime,1); %(m) Foot One, horizontal position
            D(iphase).position.footOne.y = zeros(nTime,1); %(m) Foot One, vertical position
            
            D(iphase).velocity = Velocity;
            D(iphase).velocity.footOne.x = zeros(nTime,1); %(m) Foot One, horizontal velocity
            D(iphase).velocity.footOne.y = zeros(nTime,1); %(m) Foot One, vertical velocity
            
            D(iphase).power = Power;
            D(iphase).power.ankleTwo = zeros(nTime,1); %(W) Ankle One, power consumption
            
            D(iphase).energy = Energy;
            
        case 'D' %Double stance, with Stance_Foot at origin
            
            States = output.result.interpsolution.phase(iphase).state;
            Actuators = output.result.interpsolution.phase(iphase).control;
            Parameters = P.dynamics;
            [Kinematics, Power, Energy] = kinematics_double(States, Actuators, Parameters);
            [~, contactForces] = dynamics_double(States, Actuators, Parameters);
            
            D(iphase).state = Kinematics; % Polar states all in here
            D(iphase).state.x = zeros(nTime,1); % (m) Foot One Horizontal Position
            D(iphase).state.y = zeros(nTime,1); % (m) Foot One Vertical Position
            D(iphase).state.dx = zeros(nTime,1); % (m) Foot One Horizontal Velocity
            D(iphase).state.dy = zeros(nTime,1); % (m) Foot One Vertical Velocity
            
            D(iphase).control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
            D(iphase).control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
            D(iphase).control.T1 = zeros(nTime,1); % (Nm) External torque applied to Leg One
            D(iphase).control.Thip = zeros(nTime,1); % (Nm) Hip torque applied to Leg Two from Leg One
            D(iphase).control.T2 = zeros(nTime,1); % (Nm) External torque applied to Leg Two
            
            D(iphase).contact.H1 = contactForces(:,1); %(N) Foot One, horizontal contact force
            D(iphase).contact.V1 = contactForces(:,2); %(N) Foot One, vertical contact force
            D(iphase).contact.H2 = contactForces(:,3); %(N) Foot Two, horizontal contact force
            D(iphase).contact.V2 = contactForces(:,4); %(N) Foot Two, vertical contact force
            
            [th, r] = cart2pol(D(iphase).contact.H1, D(iphase).contact.V1);
            D(iphase).contact.Mag1 = r;              %(N) Foot One, contact force magnitude
            D(iphase).contact.Ang1 = th - pi/2;      %(rad) Foot One, contact force angle from vertical
            [th, r] = cart2pol(D(iphase).contact.H2, D(iphase).contact.V2);
            D(iphase).contact.Mag2 = r;             %(N) Foot Two, contact force magnitude
            D(iphase).contact.Ang2 = th - pi/2;     %(rad) Foot Two, contact force angle from vertical
            D(iphase).contact.Bnd1 = P.ground.normal.bounds + P.ground.normal.double.one; %Bounds on the contact angle
            D(iphase).contact.Bnd2 = P.ground.normal.bounds + P.ground.normal.double.two; %Bounds on the contact angle
           
            D(iphase).position.footOne.x = zeros(nTime,1); %(m) Foot One, horizontal position
            D(iphase).position.footOne.y = zeros(nTime,1); %(m) Foot One, vertical position
            D(iphase).position.footTwo.x = Parameters.x2*ones(nTime,1); %(m) Foot Two, horizontal position
            D(iphase).position.footTwo.y = Parameters.y2*ones(nTime,1); %(m) Foot Two, vertical position
            D(iphase).position.hip.x = States(:,1); %(m) Hip, horizontal position
            D(iphase).position.hip.y = States(:,2); %(m) Hip, vertical position
            
            D(iphase).velocity.footOne.x = zeros(nTime,1); %(m) Foot One, horizontal velocity
            D(iphase).velocity.footOne.y = zeros(nTime,1); %(m) Foot One, vertical velocity
            D(iphase).velocity.footTwo.x = zeros(nTime,1); %(m) Foot Two, horizontal velocity
            D(iphase).velocity.footTwo.y = zeros(nTime,1); %(m) Foot Two, vertical velocity
            D(iphase).velocity.hip.x = States(:,3); %(m) Hip, horizontal velocity
            D(iphase).velocity.hip.y = States(:,4); %(m) Hip, vertical velocity
            
            D(iphase).power = Power;
            D(iphase).power.ankleOne = zeros(nTime,1); %(W) Ankle One, power consumption
            D(iphase).power.ankleTwo = zeros(nTime,1); %(W) Ankle Two, power consumption
            D(iphase).power.hip = zeros(nTime,1); %(W) Hip, power consumption
            
            D(iphase).energy = Energy;
            
        otherwise
            error('Unsupported Phase')
    end
end

%Go through and apply shifts necessary to line things up
for iphase=2:length(P.phase)
    D(iphase).time = D(iphase).time + D(iphase-1).time(end);
end

plotInfo.data = D;
plotInfo.parameters = P;

end