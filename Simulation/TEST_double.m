%TEST_Double
%
% Simple test of dynamics for single stance on foot one:
%
clc; clear;

Initial.data.th1 = 0.1;
Initial.data.th2 = -0.9;
Initial.data.L1 = 0.6;
Initial.data.L2 = 0.4;
Initial.data.dth1 = 0;
Initial.data.dth2 = 0;
Initial.data.dL1 = 0;
Initial.data.dL2 = 0;

Initial.data.time = 0;
Initial.data.x0 = 0.2;
Initial.data.y0 = 0.4;
Initial.data.dx0 = 0;
Initial.data.dy0 = 0;
Initial.data.x1 = -0.1;
Initial.data.y1 = 0.3;
Initial.data.dx1 = 0;
Initial.data.dy1 = 0;
Initial.data.x2 = 0.3;
Initial.data.y2 = 0.1;


Initial.duration = 5;
Initial.tolerance = 1e-10;
Initial.dataFreq = 100;

P_Dyn.m = 0.3;
P_Dyn.M = 1;
P_Dyn.g = 9.81;
Initial.parameters.dynamics = P_Dyn;

Initial.parameters.control = [];
Initial.parameters.events = [];

[Data, Final] = simulate_double(Initial);

plotInfo.data = Data;
plotInfo.parameters.dynamics = P_Dyn;
plotInfo.parameters.phase = {'D'};
plotInfo.parameters.animation.timeRate = 0.4;
plotSolution(plotInfo);
animation(plotInfo);

