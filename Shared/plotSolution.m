function plotSolution(plotInfo,figNums)

%Creates a few plots to show the states of the system.

%figNums must be an integer vector of length 3;

D = plotInfo.data;

Color_One = 'r';
Color_Two = 'b';
Color_Hip = 'm';

FontSize.title = 16;
FontSize.xlabel = 14;
FontSize.ylabel = 14;

LineWidth = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Leg Angles                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figH = figure(figNums(1)); clf;
set(figH,'Name','Leg Angles','NumberTitle','off')

%Leg One Angle
subplot(3,1,1)
    plot(D.time, D.kinematics.th1,Color_One,'LineWidth',LineWidth);
    title('Leg One Angle','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
subplot(3,1,2)
    plot(D.time, D.kinematics.th2,Color_Two,'LineWidth',LineWidth);
    title('Leg Two Angle','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
subplot(3,1,3); hold on;
    plot(D.time, D.kinematics.dth1,Color_One,'LineWidth',LineWidth);
    plot(D.time, D.kinematics.dth2,Color_Two,'LineWidth',LineWidth);
    title('Leg Angular Rates','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angular Rate (rad/s)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Leg Lengths                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(2)); clf;
set(figH,'Name','Leg Lengths','NumberTitle','off')
  
subplot(3,1,1)
    plot(D.time, D.kinematics.L1,Color_One,'LineWidth',LineWidth);
    title('Leg One Length','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
subplot(3,1,2)
    plot(D.time, D.kinematics.L2,Color_Two,'LineWidth',LineWidth);
    title('Leg Two Length','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
subplot(3,1,3); hold on;
    plot(D.time, D.kinematics.dL1,Color_One,'LineWidth',LineWidth);
    plot(D.time, D.kinematics.dL2,Color_Two,'LineWidth',LineWidth);
    title('Leg Length Rates','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Length Rate of Change (m/s)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');

    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Mass Traces                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(3)); clf; hold on;
set(figH,'Name','Point Traces','NumberTitle','off')

%Used for plotting the start (o) and end (x) markers for the traces
MarkerSize = 15;
MarkerLineWidth = 2;

plot(D.state.x0,D.state.y0,Color_Hip,'LineWidth',LineWidth);
plot(D.state.x1,D.state.y1,Color_One,'LineWidth',LineWidth);
plot(D.state.x2,D.state.y2,Color_Two,'LineWidth',LineWidth);
xlabel('Horizontal Position (m)','FontSize',FontSize.xlabel)
ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
title('Point Mass Traces','FontSize',FontSize.title)

%Plot the start and end of each trace.
plot(D.state.x0(1),D.state.y0(1),[Color_Hip 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
plot(D.state.x1(1),D.state.y1(1),[Color_One 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
plot(D.state.x2(1),D.state.y2(1),[Color_Two 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
plot(D.state.x0(end),D.state.y0(end),[Color_Hip 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
plot(D.state.x1(end),D.state.y1(end),[Color_One 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
plot(D.state.x2(end),D.state.y2(end),[Color_Two 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);

legend('Hip','Foot One','Foot Two')
axis equal

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Position and Velocity                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(4)); clf;
set(figH,'Name','Kinematics','NumberTitle','off')

subplot(3,1,1); hold on;
    plot(D.time,D.state.x0,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,D.state.x1,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.state.x2,Color_Two,'LineWidth',LineWidth);
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Horizontal Position (m)','FontSize',FontSize.ylabel)
    title('Point Mass Horizontal Positions','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')

subplot(3,1,2); hold on;
    plot(D.time,D.state.y0,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,D.state.y1,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.state.y2,Color_Two,'LineWidth',LineWidth);
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
    title('Point Mass Vertical Positions','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')    
    
subplot(3,1,3); hold on;
    Speed.hip = sqrt(D.state.dx0.^2 + D.state.dy0.^2);
    Speed.one = sqrt(D.state.dx1.^2 + D.state.dy1.^2);
    Speed.two = sqrt(D.state.dx2.^2 + D.state.dy2.^2);
    plot(D.time,Speed.hip,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,Speed.one,Color_One,'LineWidth',LineWidth);
    plot(D.time,Speed.two,Color_Two,'LineWidth',LineWidth);
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Speed (m/s)','FontSize',FontSize.ylabel)
    title('Point Mass Speed','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')    
    
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Actuators                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(5)); clf;
set(figH,'Name','Actuators','NumberTitle','off')

subplot(3,1,1); hold on;
    plot(D.time, D.control.F1, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.control.F2, Color_Two,'LineWidth',LineWidth);
    title('Leg Axial Force','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Force (N)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
subplot(3,1,2); hold on;
    plot(D.time, D.control.T1, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.control.T2, Color_Two,'LineWidth',LineWidth);
    title('Angle Torque','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
subplot(3,1,3)
    plot(D.time, D.control.Thip,Color_Hip,'LineWidth',LineWidth);
    title('Hip Torque','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Power                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(6)); clf;
set(figH,'Name','Power','NumberTitle','off')

subplot(3,1,1); hold on;
    plot(D.time, D.power.legOne, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.power.legTwo, Color_Two,'LineWidth',LineWidth);
    title('Leg Axial Power','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
subplot(3,1,2); hold on;
    plot(D.time, D.power.ankleOne, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.power.ankleTwo, Color_Two,'LineWidth',LineWidth);
    title('Ankle Power','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
subplot(3,1,3)
    plot(D.time, D.power.hip,Color_Hip,'LineWidth',LineWidth);
    title('Hip Power','FontSize',FontSize.title)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    
    

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Contact Forces                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(figNums(7)); clf;
set(figH,'Name','Contact Forces','NumberTitle','off')   

subplot(2,1,1); hold on;
    plot(D.time,D.contacts.F1_ang,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.contacts.F2_ang,Color_Two,'LineWidth',LineWidth);
    title('Contact Force Angle','FontSize',FontSize.title)
    ylabel('Angle from vertical (rad)','FontSize',FontSize.ylabel)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    legend('Foot One', 'Foot Two')
    
subplot(2,1,2); hold on;
    plot(D.time,D.contacts.F1_mag,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.contacts.F2_mag,Color_Two,'LineWidth',LineWidth);
    title('Contact Force Magnitude','FontSize',FontSize.title)
    ylabel('Force (N)','FontSize',FontSize.ylabel)
     xlabel('Time (s)','FontSize',FontSize.xlabel)
    legend('Foot One', 'Foot Two')

end

