function plotInfo = getPlotInfo(output)

%This function formats the output from gpops2 and does some additional
%calculations for kinematics, energy, power, ect.

auxDat = output.result.setup.auxdata;

nPhase = length(auxDat.phase);
plotInfo.data = zeros(nPhase,0);
plotInfo.parameters = auxDat;

for iphase=1:nPhase;
    
    
    Phase = auxDat.phase{iphase};
    
    plotInfo.data(iphase).phase = Phase;
    
    %Format things for plotting:
    States = output.result.solution.phase(iphase).state;
    Actuators = output.result.solution.phase(iphase).control;
    
    Cst = constraints(States, Actuators, Phase, auxDat.parameters);
    
    %Copy the existing solution over to the output
    plotInfo.data(iphase).time = output.result.solution.phase(iphase).time;
    
    stepVec = auxDat.parameters.StepVector;
    nTime = length(output.result.solution.phase(iphase).time);
    switch Phase
        case 'S1'
            
            plotInfo.data(iphase).state.x0 = States(:,1);
            plotInfo.data(iphase).state.y0 = States(:,2);
            plotInfo.data(iphase).state.x2 = States(:,3);
            plotInfo.data(iphase).state.y2 = States(:,4);
            plotInfo.data(iphase).state.dx0 = States(:,5);
            plotInfo.data(iphase).state.dy0 = States(:,6);
            plotInfo.data(iphase).state.dx2 = States(:,7);
            plotInfo.data(iphase).state.dy2 = States(:,8);
            
            plotInfo.data(iphase).state.x1 = zeros(nTime,1);
            plotInfo.data(iphase).state.y1 = zeros(nTime,1);
            plotInfo.data(iphase).state.dx1 = zeros(nTime,1);
            plotInfo.data(iphase).state.dy1 = zeros(nTime,1);
            
            plotInfo.data(iphase).control.H1 = Actuators(:,1);
            plotInfo.data(iphase).control.V1 = Actuators(:,2);
            plotInfo.data(iphase).control.H2 = Actuators(:,3);
            plotInfo.data(iphase).control.V2 = Actuators(:,4);
            
            plotInfo.data(iphase).constraint.L1 = sqrt(Cst(:,1));
            plotInfo.data(iphase).constraint.L2 = sqrt(Cst(:,2));
            plotInfo.data(iphase).constraint.Contact_Angle_One = Cst(:,3);
            plotInfo.data(iphase).constraint.Contact_Angle_Two = zeros(size(Cst(:,3)));
            
        case 'D'
            
            plotInfo.data(iphase).state.x0 = States(:,1);
            plotInfo.data(iphase).state.y0 = States(:,2);
            plotInfo.data(iphase).state.dx0 = States(:,3);
            plotInfo.data(iphase).state.dy0 = States(:,4);
            
            plotInfo.data(iphase).state.x2 = -stepVec(1)*ones(nTime,1);
            plotInfo.data(iphase).state.y2 = -stepVec(2)*ones(nTime,1);
            plotInfo.data(iphase).state.dx2 = zeros(nTime,1);
            plotInfo.data(iphase).state.dy2 = zeros(nTime,1);
            
            plotInfo.data(iphase).state.x1 = zeros(nTime,1);
            plotInfo.data(iphase).state.y1 = zeros(nTime,1);
            plotInfo.data(iphase).state.dx1 = zeros(nTime,1);
            plotInfo.data(iphase).state.dy1 = zeros(nTime,1);
            
            plotInfo.data(iphase).control.H1 = Actuators(:,1);
            plotInfo.data(iphase).control.V1 = Actuators(:,2);
            plotInfo.data(iphase).control.H2 = Actuators(:,3);
            plotInfo.data(iphase).control.V2 = Actuators(:,4);
            
            plotInfo.data(iphase).constraint.L1 = sqrt(Cst(:,1));
            plotInfo.data(iphase).constraint.L2 = sqrt(Cst(:,2));
            plotInfo.data(iphase).constraint.Contact_Angle_One = Cst(:,3);
            plotInfo.data(iphase).constraint.Contact_Angle_Two = Cst(:,4);
            
        otherwise
            error('Invalid Phase')
    end
end

end