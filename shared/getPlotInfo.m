function plotInfo = getPlotInfo(output)

P = output.result.setup.auxdata;
nPhase = length(P.phase);
D = zeros(nPhase,0);

for iphase = 1:length(P.phase)
    
    time = output.result.interpsolution.phase(iphase).time;
    phase = P.phase{iphase};
    nTime = length(time);
    
    switch phase
        case 'S1' %Single stance, with Stance_Foot == Foot_One
            
            IdxState = 1:8;
            IdxAct = 9:12;
            IdxActRate = 1:4;
            IdxAbsPos = 5:8;
            IdxAbsNeg = 9:12;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
            Data = formatData(time, phase, States, Actuators, Parameters);
            names = fieldnames(Data);
            for i=1:length(names)
                D(iphase).(names{i}) = Data.(names{i});
            end
            
            Step_Dist = output.result.solution.parameter;
            
            [~, n1] = feval(P.ground.func,0,[]);
            D(iphase).contact.Bnd1 = P.ground.normal.bounds + n1; %Bounds on the contact angle
            D(iphase).contact.Bnd2 = zeros(2,1);
            
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                P.dynamics,P.cost , AbsPos, AbsNeg, D(iphase).phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
        case 'D' %Double stance, with Stance_Foot at origin
            
            D(iphase).time = time;
            D(iphase).phase = phase;
            
            IdxState = 1:4;
            IdxAct = 5:6;
            IdxActRate = 1:2;
            IdxAbsPos = 3:4;
            IdxAbsNeg = 5:6;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
            Step_Dist = output.result.solution.parameter;
            Parameters.x2 = -Step_Dist;
            [Parameters.y2, n2] = feval(P.ground.func,-Step_Dist,[]);
            [~, n1] = feval(P.ground.func,0,[]);
            
             Data = formatData(time, phase, States, Actuators, Parameters);
            names = fieldnames(Data);
            for i=1:length(names)
                D(iphase).(names{i}) = Data.(names{i});
            end
            
            D(iphase).contact.Bnd1 = P.ground.normal.bounds + n1; %Bounds on the contact angle
            D(iphase).contact.Bnd2 = P.ground.normal.bounds + n2; %Bounds on the contact angle
            
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                Parameters,P.cost , AbsPos, AbsNeg, phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
        case 'F' %Single stance, with Stance_Foot == Foot_One
                        
            error('NEED TO MOVE THIS TO formatData.m');
            
            D(iphase).time = time;
            D(iphase).phase = phase;
            
            IdxState = 1:12;
            IdxAct = 13:15;
            IdxActRate = 1:3;
            IdxAbsPos = 4:6;
            IdxAbsNeg = 7:9;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
            Step_Dist = output.result.solution.parameter;
            
            D(iphase).state.th1 = States(:,3); % (rad) Leg One absolute angle
            D(iphase).state.th2 = States(:,4); % (rad) Leg Two absolute angle
            D(iphase).state.L1 = States(:,5); % (m) Leg One length
            D(iphase).state.L2 = States(:,6); % (m) Leg Two length
            D(iphase).state.dth1 = States(:,9); % (rad/s) Leg One absolute angular rate
            D(iphase).state.dth2 = States(:,10); % (rad/s) Leg Two absolute angular rate
            D(iphase).state.dL1 = States(:,11); % (m/s) Leg One extension rate
            D(iphase).state.dL2 = States(:,12); % (m/s) Leg Two extensioin rate
            D(iphase).state.x = States(:,1); % (m) Foot One Horizontal Position
            D(iphase).state.y = States(:,2); % (m) Foot One Vertical Position
            D(iphase).state.dx = States(:,7); % (m) Foot One Horizontal Velocity
            D(iphase).state.dy = States(:,8); % (m) Foot One Vertical Velocity
            
            D(iphase).control.F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
            D(iphase).control.F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
            D(iphase).control.T1 = zeros(nTime,1); % (Nm) External torque applied to Leg One
            D(iphase).control.Thip = Actuators(:,3); % (Nm) Hip torque applied to Leg Two from Leg One
            D(iphase).control.T2 = zeros(nTime,1); % (Nm) External torque applied to Leg Two
            
            D(iphase).contact.H1 = zeros(nTime,1); %(N) Foot One, horizontal contact force
            D(iphase).contact.V1 = zeros(nTime,1); %(N) Foot One, vertical contact force
            D(iphase).contact.H2 = zeros(nTime,1); %(N) Foot Two, horizontal contact force
            D(iphase).contact.V2 = zeros(nTime,1); %(N) Foot Two, vertical contact force
            
            D(iphase).contact.Mag1 = zeros(nTime,1);              %(N) Foot One, contact force magnitude
            D(iphase).contact.Ang1 = zeros(nTime,1);      %(rad) Foot One, contact force angle from vertical
            D(iphase).contact.Mag2 = zeros(nTime,1); %(N) Foot Two, contact force magnitude
            D(iphase).contact.Ang2 = zeros(nTime,1); %(rad) Foot Two, contact force angle from vertical
            D(iphase).contact.Bnd1 = zeros(2,1);
            D(iphase).contact.Bnd2 = zeros(2,1);
            
            [Position, Velocity, Power, Energy] = kinematics_flight(States, Actuators, Parameters);
            D(iphase).position = Position;
            D(iphase).velocity = Velocity;
            
            D(iphase).power = Power;
            D(iphase).power.ankleOne = zeros(nTime,1); %(W) Ankle One, power consumption
            D(iphase).power.ankleTwo = zeros(nTime,1); %(W) Ankle Two, power consumption
            
            D(iphase).energy = Energy;
            
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                P.dynamics,P.cost , AbsPos, AbsNeg, D(iphase).phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
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