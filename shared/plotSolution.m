function plotSolution(plotInfo,figNums)

%Creates a few plots to show the states of the system.

%figNums must be an integer vector of length 3;

Color_One = 'r';
Color_Two = 'b';
Color_Hip = 'm';

FontSize.title = 16;
FontSize.xlabel = 14;
FontSize.ylabel = 14;

LineWidth = 2;

%Clear all of the figures
for i=1:length(figNums)
    figure(figNums(i)); clf;
end

PHASE = plotInfo.parameters.phase;
energyDatum = [];
for iphase = 1:length(PHASE)
    
    D = plotInfo.data(iphase);
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                           Leg Angles                                    %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
    figH = figure(figNums(1));
    set(figH,'Name','Leg Angles','NumberTitle','off')
    
    %Leg One Angle
    subplot(3,1,1); hold on;
    plot(D.time, D.state.th1,Color_One,'LineWidth',LineWidth);
    title('Leg One Angle','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,2); hold on;
    plot(D.time, D.state.th2,Color_Two,'LineWidth',LineWidth);
    title('Leg Two Angle','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angle (rad)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,3); hold on;
    plot(D.time, D.state.dth1,Color_One,'LineWidth',LineWidth);
    plot(D.time, D.state.dth2,Color_Two,'LineWidth',LineWidth);
    title('Leg Angular Rates','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Angular Rate (rad/s)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                         Leg Lengths                                     %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(2));
    set(figH,'Name','Leg Lengths','NumberTitle','off')
    
    subplot(3,1,1); hold on;
    plot(D.time, D.state.L1,Color_One,'LineWidth',LineWidth);
    title('Leg One Length','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Length (m)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,2); hold on;
    plot(D.time, D.state.L2,Color_Two,'LineWidth',LineWidth);
    title('Leg Two Length','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Length (m)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,3); hold on;
    plot(D.time, D.state.dL1,Color_One,'LineWidth',LineWidth);
    plot(D.time, D.state.dL2,Color_Two,'LineWidth',LineWidth);
    title('Leg Length Rates','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Length Rate of Change (m/s)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                         Mass Traces                                     %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(3)); hold on;
    set(figH,'Name','Point Traces','NumberTitle','off')
    
    %Used for plotting the start (o) and end (x) markers for the traces
    MarkerSize = 15;
    MarkerLineWidth = 2;
    
    Kin.x0 = D.position.hip.x;
    Kin.y0 = D.position.hip.y;
    Kin.dx0 = D.velocity.hip.x;
    Kin.dy0 = D.velocity.hip.y;
    Kin.x1 = D.position.footOne.x;
    Kin.y1 = D.position.footOne.y;
    Kin.dx1 = D.velocity.footOne.x;
    Kin.dy1 = D.velocity.footOne.y;
    Kin.x2 = D.position.footTwo.x;
    Kin.y2 = D.position.footTwo.y;
    Kin.dx2 = D.velocity.footTwo.x;
    Kin.dy2 = D.velocity.footTwo.y;
    
    plot(Kin.x0,Kin.y0,Color_Hip,'LineWidth',LineWidth);
    plot(Kin.x1,Kin.y1,Color_One,'LineWidth',LineWidth);
    plot(Kin.x2,Kin.y2,Color_Two,'LineWidth',LineWidth);
    xlabel('Horizontal Position (m)','FontSize',FontSize.xlabel)
    ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
    title('Point Mass Traces','FontSize',FontSize.title)
    
    %Plot the start and end of each trace.
    plot(Kin.x0(1),Kin.y0(1),[Color_Hip 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    plot(Kin.x1(1),Kin.y1(1),[Color_One 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    plot(Kin.x2(1),Kin.y2(1),[Color_Two 'o'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    plot(Kin.x0(end),Kin.y0(end),[Color_Hip 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    plot(Kin.x1(end),Kin.y1(end),[Color_One 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    plot(Kin.x2(end),Kin.y2(end),[Color_Two 'x'],'MarkerSize',MarkerSize,'LineWidth',MarkerLineWidth);
    
    legend('Hip','Foot One','Foot Two')
    axis equal
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                     Position and Velocity                               %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(4));
    set(figH,'Name','Kinematics','NumberTitle','off')
    
    subplot(3,1,1); hold on;
    plot(D.time,Kin.x0,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,Kin.x1,Color_One,'LineWidth',LineWidth);
    plot(D.time,Kin.x2,Color_Two,'LineWidth',LineWidth);
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Horizontal Position (m)','FontSize',FontSize.ylabel)
    title('Point Mass Horizontal Positions','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')
    dottedLine(D.time(1),axis,iphase);
    
    subplot(3,1,2); hold on;
    plot(D.time,Kin.y0,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,Kin.y1,Color_One,'LineWidth',LineWidth);
    plot(D.time,Kin.y2,Color_Two,'LineWidth',LineWidth);
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
    title('Point Mass Vertical Positions','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')
    dottedLine(D.time(1),axis,iphase);
    
    subplot(3,1,3); hold on;
    Speed.hip = sqrt(Kin.dx0.^2 + Kin.dy0.^2);
    Speed.one = sqrt(Kin.dx1.^2 + Kin.dy1.^2);
    Speed.two = sqrt(Kin.dx2.^2 + Kin.dy2.^2);
    plot(D.time,Speed.hip,Color_Hip,'LineWidth',LineWidth);
    plot(D.time,Speed.one,Color_One,'LineWidth',LineWidth);
    plot(D.time,Speed.two,Color_Two,'LineWidth',LineWidth);
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Speed (m/s)','FontSize',FontSize.ylabel)
    title('Point Mass Speed','FontSize',FontSize.title)
    legend('Hip','Foot One','Foot Two')
    dottedLine(D.time(1),axis,iphase);
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                              Actuators                                  %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(5));
    set(figH,'Name','Actuators','NumberTitle','off')
    
    subplot(3,1,1); hold on;
    plot(D.time, D.control.F1, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.control.F2, Color_Two,'LineWidth',LineWidth);
    title('Leg Axial Force','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Force (N)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,2); hold on;
    plot(D.time, D.control.T1, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.control.T2, Color_Two,'LineWidth',LineWidth);
    title('Angle Torque','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,3); hold on;
    plot(D.time, D.control.Thip,Color_Hip,'LineWidth',LineWidth);
    title('Hip Torque','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                              Power                                      %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(6));
    set(figH,'Name','Power','NumberTitle','off')
    
    subplot(3,1,1); hold on;
    plot(D.time, D.power.legOne, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.power.legTwo, Color_Two,'LineWidth',LineWidth);
    title('Leg Axial Power','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,2); hold on;
    plot(D.time, D.power.ankleOne, Color_One,'LineWidth',LineWidth);
    plot(D.time, D.power.ankleTwo, Color_Two,'LineWidth',LineWidth);
    title('Ankle Power','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    legend('Leg One','Leg Two');
    dottedLine(D.time(1),axis,iphase);
    subplot(3,1,3); hold on;
    plot(D.time, D.power.hip,Color_Hip,'LineWidth',LineWidth);
    title('Hip Power','FontSize',FontSize.title)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    ylabel('Power (W)','FontSize',FontSize.ylabel)
    dottedLine(D.time(1),axis,iphase);
    
    
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                         Contact Forces                                  %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(7));
    set(figH,'Name','Contact Forces','NumberTitle','off')
    
    subplot(2,1,1); hold on;
    plot(D.time,D.contact.Ang1,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.contact.Ang2,Color_Two,'LineWidth',LineWidth);
    title('Contact Force Angle','FontSize',FontSize.title)
    ylabel('Angle from vertical (rad)','FontSize',FontSize.ylabel)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    legend('Foot One', 'Foot Two')
    dottedLine(D.time(1),axis,iphase);
    
    subplot(2,1,2); hold on;
    plot(D.time,D.contact.Mag1,Color_One,'LineWidth',LineWidth);
    plot(D.time,D.contact.Mag2,Color_Two,'LineWidth',LineWidth);
    title('Contact Force Magnitude','FontSize',FontSize.title)
    ylabel('Force (N)','FontSize',FontSize.ylabel)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    legend('Foot One', 'Foot Two')
    dottedLine(D.time(1),axis,iphase);
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                         System Energy                                   %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    figH = figure(figNums(8));
    set(figH,'Name','System Energy','NumberTitle','off')
    hold on;
    if isempty(energyDatum)
        energyDatum = mean(D.energy.Potential);
    end
    plot(D.time,D.energy.Kinetic,'k:','LineWidth',LineWidth+1)
    plot(D.time,D.energy.Potential - energyDatum,'k--','LineWidth',LineWidth+1)
    plot(D.time,D.energy.Total - energyDatum,'k-','LineWidth',LineWidth+2)
    title('System Energy','FontSize',FontSize.title)
    ylabel('Energy (J)','FontSize',FontSize.ylabel)
    xlabel('Time (s)','FontSize',FontSize.xlabel)
    legend('Kinetic','Potential','Total')
    dottedLine(D.time(1),axis,iphase);
    
end %iphase

end

%%%% SUB FUNCTIONS %%%%

function dottedLine(time,AXIS,iphase)

if iphase>1
    %Plots a dotted line between phases
    plot(time*[1;1],[AXIS(3);AXIS(4)],'k:','LineWidth',1);
end

end

