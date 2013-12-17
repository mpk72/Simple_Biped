function plotInfo = getPlotInfo(output)

%This function formats the output from gpops2 and does some additional
%calculations for kinematics, energy, power, ect.

% For now, use matrix convention: [Ndata, Ntime]

nPhase = length(output.result.setup.auxdata.phase);
plotInfo.data = zeros(nPhase,0);
plotInfo.parameters = output.result.setup.auxdata;

for iphase=1:nPhase;
    
    %Store the code for the current phase of motion.
    try
        plotInfo.data(iphase).phase(iphase) = plotInfo.parameters.phase(iphase);
    catch ME
       disp('--> Make sure that auxdata.phase(iphase) exists');
       disp('--> This should contain a value in the set {''D'',''S1'',''S2'',''F''}')
       throw(ME);
    end
    
    %Format things for plotting:
    stateDat = output.result.solution.phase(iphase).state;
    actDat = output.result.solution.phase(iphase).control;
    phaseDat = output.result.setup.auxdata.state;
phaseDat.phase = 'D';
phaseDat.mode = 'gpops_to_dynamics';
    [States, Actuators] = DataRestructure(stateDat,phaseDat,actDat);
    
    %Copy the existing solution over to the output
      plotInfo.data(iphase).time = output.result.solution.phase(iphase).time;
      plotInfo.data(iphase).state = convert(States);
      plotInfo.data(iphase).control = convert(Actuators);
      plotInfo.data(iphase).integral = output.result.solution.phase(iphase).integral;
         
    %Get kinematics and energy info:
    Parameters = output.result.setup.auxdata.dynamics;
    Kinematics = kinematics(States);
    plotInfo.data(iphase).kinematics = Kinematics;
    plotInfo.data(iphase).energy = energy(States, Parameters);
    
    %Get the power used by the actuators:
    Actuators = output.result.solution.phase(iphase).control;
    Phase = plotInfo.data(iphase).phase(iphase);
    plotInfo.data(iphase).power = actuatorPower(States, Actuators, Phase);
    
end






end