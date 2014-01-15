function Actuators = control_single(States, P_Ctl)

%Extract states:
L1 = States(:,3); % (m) Leg One length
L2 = States(:,4); % (m) Leg Two length
dL1 = States(:,7); % (m/s) Leg One extension rate
dL2 = States(:,8); % (m/s) Leg Two extensioin rate

%For now, just do a PD controller on length:
Kp1 = -10;
Kd1 = 0;
Lo1 = 0.5;

Kp2 = -10;
Kd2 = 0;
Lo2 = 0.5;

F1 = Kp1*(L1 - Lo1) + Kd1*dL1;
F2 = Kp2*(L2 - Lo2) + Kd2*dL2;

n = length(F1);

Actuators = [F1, F2, zeros(n,2)];

end