function plotSolution(plotInfo,figNums)

%Creates a few plots to show the states of the system.

%figNums must be an integer vector of length 3;

D = plotInfo.data;

Color_One = 'r';
Color_Two = 'b';
Color_Hip = 'm';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Leg Angles                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(figNums(1)); clf;

%Leg One Angle
subplot(3,1,1)
    plot(D.time, D.kinematics.th1,Color_One);
    title('Leg One Angle')
    xlabel('Time (s)')
    ylabel('Angle (rad)')
subplot(3,1,2)
    plot(D.time, D.kinematics.th2,Color_Two);
    title('Leg Two Angle')
    xlabel('Time (s)')
    ylabel('Angle (rad)')
subplot(3,1,3); hold on;
    plot(D.time, D.kinematics.dth1,Color_One);
    plot(D.time, D.kinematics.dth2,Color_Two);
    title('Leg Angular Rates')
    xlabel('Time (s)')
    ylabel('Angular Rate (rad/s)')
    legend('Leg One','Leg Two');
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Leg Lengths                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figure(figNums(2)); clf;
  
subplot(3,1,1)
    plot(D.time, D.kinematics.L1,Color_One);
    title('Leg One Length')
    xlabel('Time (s)')
    ylabel('Angle (rad)')
subplot(3,1,2)
    plot(D.time, D.kinematics.L2,Color_Two);
    title('Leg Two Length')
    xlabel('Time (s)')
    ylabel('Angle (rad)')
subplot(3,1,3); hold on;
    plot(D.time, D.kinematics.dL1,Color_One);
    plot(D.time, D.kinematics.dL2,Color_Two);
    title('Leg Length Rates')
    xlabel('Time (s)')
    ylabel('Length Rate of Change (m/s)')
    legend('Leg One','Leg Two');

    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Mass Traces                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figure(figNums(3)); clf; hold on;

plot(D.state.x0,D.state.y0,Color_Hip);
plot(D.state.x1,D.state.y1,Color_One);
plot(D.state.x2,D.state.y2,Color_Two);
xlabel('Horizontal Position (m)')
ylabel('Vertical Position (m)')
title('Point Mass Traces')
legend('Hip','Foot One','Foot Two')
axis equal

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Position and Velocity                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figure(figNums(4)); clf;

subplot(3,1,1); hold on;
    plot(D.time,D.state.x0,Color_Hip);
    plot(D.time,D.state.x1,Color_One);
    plot(D.time,D.state.x2,Color_Two);
    xlabel('Time (s)')
    ylabel('Horizontal Position (m)')
    title('Point Mass Horizontal Positions')
    legend('Hip','Foot One','Foot Two')

subplot(3,1,2); hold on;
    plot(D.time,D.state.y0,Color_Hip);
    plot(D.time,D.state.y1,Color_One);
    plot(D.time,D.state.y2,Color_Two);
    xlabel('Time (s)')
    ylabel('Vertical Position (m)')
    title('Point Mass Vertical Positions')
    legend('Hip','Foot One','Foot Two')    
    
subplot(3,1,3); hold on;
    Speed.hip = sqrt(D.state.dx0.^2 + D.state.dy0.^2);
    Speed.one = sqrt(D.state.dx1.^2 + D.state.dy1.^2);
    Speed.two = sqrt(D.state.dx2.^2 + D.state.dy2.^2);
    plot(D.time,Speed.hip,Color_Hip);
    plot(D.time,Speed.one,Color_One);
    plot(D.time,Speed.two,Color_Two);
    xlabel('Time (s)')
    ylabel('Speed (m/s)')
    title('Point Mass Speed')
    legend('Hip','Foot One','Foot Two')    
    
    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Actuators                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figure(figNums(5)); clf;

%Leg One Angle
subplot(3,1,1); hold on;
    plot(D.time, D.control.F1, Color_One);
    plot(D.time, D.control.F2, Color_Two);
    title('Leg Axial Force')
    xlabel('Time (s)')
    ylabel('Force (N)')
    legend('Leg One','Leg Two');
subplot(3,1,2); hold on;
    plot(D.time, D.control.T1, Color_One);
    plot(D.time, D.control.T2, Color_Two);
    title('Angle Torque')
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    legend('Leg One','Leg Two');
subplot(3,1,3)
    plot(D.time, D.control.Thip,Color_Hip);
    title('Hip Torque')
    xlabel('Time (s)')
    ylabel('Torque (Nm)')

end

