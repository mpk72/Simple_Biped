function plotInfo = getPlotInfo(output)

%This function formats the output from gpops2 and does some additional
%calculations for kinematics, energy, power, ect.

% For now, use matrix convention: [Ndata, Ntime]

nPhase = length(output.result.setup.auxdata.phase);
plotInfo.data = zeros(nPhase,0);
plotInfo.parameters = output.result.setup.auxdata;

Step_Length = output.result.solution.parameter;
Slope = output.result.setup.auxdata.misc.Ground_Slope;

for iphase=1:nPhase;
    
    %Store the code for the current phase of motion.
    try
        Phase = output.result.setup.auxdata.phase{iphase};
    catch ME
        disp('--> Make sure that auxdata.phase(iphase) exists');
        disp('--> This should contain a value in the set {''D'',''S1'',''S2'',''F''}')
        throw(ME);
    end
    
    plotInfo.data(iphase).phase = Phase;
    
    %Format things for plotting:
    stateDat = output.result.solution.phase(iphase).state;
    actDat = output.result.solution.phase(iphase).control;
    
    switch iphase
        case 1
            phaseDat.x1 = Step_Length*cos(Slope);
            phaseDat.y1 = Step_Length*sin(Slope);
            phaseDat.x2 = zeros(size(Step_Length));
            phaseDat.y2 = zeros(size(Step_Length));
        case 2
            
            phaseDat.x1 = Step_Length*cos(Slope);
            phaseDat.y1 = Step_Length*sin(Slope);
        otherwise
            error('Phase not supported!')
    end

    phaseDat.phase = Phase;
    phaseDat.mode = 'gpops_to_dynamics';
    [States, Actuators] = DataRestructure(stateDat,phaseDat,actDat);
    
    %Get the contact forces:
    [~, Contacts] = dynamics(States,Actuators,plotInfo.parameters.dynamics,Phase);
    
    %Copy the existing solution over to the output
    plotInfo.data(iphase).time = output.result.solution.phase(iphase).time;
    plotInfo.data(iphase).state = convert(States);
    plotInfo.data(iphase).control = convert(Actuators);
    plotInfo.data(iphase).integral = output.result.solution.phase(iphase).integral;
    plotInfo.data(iphase).contacts = convert(Contacts);
    
    
    %Compute extra information about contacts:
    C = plotInfo.data(iphase).contacts;
    plotInfo.data(iphase).contacts.F1_ang = atan2(C.H1,C.V1);
    plotInfo.data(iphase).contacts.F2_ang = atan2(C.H2,C.V2);
    plotInfo.data(iphase).contacts.F1_mag = sqrt(C.H1.^2 + C.V1.^2);
    plotInfo.data(iphase).contacts.F2_mag = sqrt(C.H2.^2 + C.V2.^2);
    
    
    %Get kinematics and energy info:
    Parameters = output.result.setup.auxdata.dynamics;
    Kinematics = kinematics(States);
    plotInfo.data(iphase).kinematics = Kinematics;
    plotInfo.data(iphase).energy = energy(States, Parameters);
    
    %Get the power used by the actuators:
    plotInfo.data(iphase).power = actuatorPower(States, Actuators, Phase);
    
end

end