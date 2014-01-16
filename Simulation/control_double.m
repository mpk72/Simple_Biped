function Actuators = control_double(States, P_Dyn, P_Ctl)

Kinematics = kinematics_double(States, [], P_Dyn);

%Extract states:
L1 = Kinematics.L1; % (m) Leg One length
L2 = Kinematics.L2; % (m) Leg Two length
dL1 = Kinematics.dL1; % (m/s) Leg One extension rate
dL2 = Kinematics.dL2; % (m/s) Leg Two extensioin rate

%For now, just do a PD controller on length:
Kp1 = -100;
Kd1 = -5;
Lo1 = 0.5;

Kp2 = -30;
Kd2 = -4;
Lo2 = 0.5;

F1 = Kp1*(L1 - Lo1) + Kd1*dL1;
F2 = Kp2*(L2 - Lo2) + Kd2*dL2;

n = length(F1);

Actuators = -[F1, F2, zeros(n,1)];

end