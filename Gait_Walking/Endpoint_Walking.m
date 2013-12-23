function output = Endpoint_Walking(input)

Step_Length = input.parameter;
Slope = input.auxdata.misc.Ground_Slope;

%%%% HACK %%%%

%This code implicitly assumes Slope = zero for defect calculations
if Slope ~= 0
    error('Non-zero slope is not allowed for now')
end

%%%% DONE %%%%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 1  --  D  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;
phaseDat.x1 = Step_Length*cos(Slope);
phaseDat.y1 = Step_Length*sin(Slope);
phaseDat.x2 = zeros(size(Step_Length));
phaseDat.y2 = zeros(size(Step_Length));

%Defect between phase 1 and 2
phaseDat.mode = 'gpops_to_dynamics';
phaseDat.phase = 'D';
LAST = convert(DataRestructure(input.phase(iphase).finalstate,phaseDat));
phaseDat.phase = 'S1';
FIRST = convert(DataRestructure(input.phase(iphase+1).initialstate,phaseDat));
event.defect_12 =[...
    LAST.x2 - FIRST.x2,...
    LAST.y2 - FIRST.y2,...
    LAST.x0 - FIRST.x0,...
    LAST.y0 - FIRST.y0,...
    LAST.dx0 - FIRST.dx0,...
    LAST.dy0 - FIRST.dy0,...
    LAST.dx2 - FIRST.dx2,...
    LAST.dy2 - FIRST.dy2,...
    ];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              PHASE 2  --  S1  --  Single Stance One                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;
phaseDat.x1 = Step_Length*cos(Slope);
phaseDat.y1 = Step_Length*sin(Slope);

%Periodic constraint
phaseDat.mode = 'gpops_to_dynamics';
phaseDat.phase = 'S1';
LAST = convert(DataRestructure(input.phase(iphase).finalstate,phaseDat));
phaseDat.phase = 'D';
FIRST = convert(DataRestructure(input.phase(1).initialstate,phaseDat));

%Note that the feet switch
%Everything maps exactly, except for horizontal translation
event.periodic = [...
    (LAST.x1 - LAST.x2) - (FIRST.x2 - FIRST.x1),...
    (LAST.x0 - LAST.x2) - (FIRST.x0 - FIRST.x1),...
    LAST.y0 - FIRST.y0,...
    LAST.y1 - FIRST.y2,...
    LAST.y2 - FIRST.y1,...
    LAST.dx0 - FIRST.dx0,...
    LAST.dy0 - FIRST.dy0,...
    ];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Other Constraints                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Require that things be monotonic in time
event.time = input.phase(2).initialtime - input.phase(1).finaltime;

output.eventgroup = packConstraints(event,'event_walking');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
if strcmp(input.auxdata.cost.method,'Squared')
    output.objective = input.phase(1).integral + input.phase(2).integral;
else
    error('Unsupported Cost Function');
end

end