%TEST_Single_One
%
% Simple test of dynamics for single stance on foot one:
%

Initial.data.th1 = 0.1;
Initial.data.th2 = -0.1;
Initial.data.L1 = 0.6;
Initial.data.L2 = 0.4;
Initial.data.dth1 = 0;
Initial.data.dth2 = 0;
Initial.data.dL1 = 0;
Initial.data.dL2 = 0;

Initial.data.time = 0;
Initial.data.x1 = 0;
Initial.data.y1 = 0;
Initial.data.dx1 = 0;
Initial.data.dy1 = 0;

Initial.duration = 2;
Initial.tolerance = 1e-10;
Initial.dataFreq = 100;

P_Dyn.m = 0.3;
P_Dyn.M = 1;
P_Dyn.g = 9.81;
Initial.parameters.dynamics = P_Dyn;

Initial.parameters.control = [];
Initial.parameters.events = [];

[Data, Final] = simulate_single_one(Initial);