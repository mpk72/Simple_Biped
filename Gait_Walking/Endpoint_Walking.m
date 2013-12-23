function output = Endpoint_Walking(input)

StepLength = input.parameter;
Slope = input.auxdata.misc.Ground_Slope;

%First (in time) stance foot location
StepVec_x = StepLength*cos(Slope);
StepVec_y = StepLength*sin(Slope);

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
phaseDat.x1 = StepVec_x;
phaseDat.y1 = StepVec_y;
phaseDat.x2 = zeros(size(StepLength));
phaseDat.y2 = zeros(size(StepLength));

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
phaseDat.x1 = StepVec_x;
phaseDat.y1 = StepVec_y;

%Periodic constraint
phaseDat.mode = 'gpops_to_dynamics';
phaseDat.phase = 'S1';
LAST = convert(DataRestructure(input.phase(iphase).finalstate,phaseDat));
phaseDat.phase = 'D';
FIRST = convert(DataRestructure(input.phase(1).initialstate,phaseDat));

%Note that the feet switch
event.periodic = [...
    (LAST.x2 - FIRST.x1) - StepVec_x,...
    (LAST.y2 - FIRST.y1) - StepVec_y,...
    (LAST.x1 - FIRST.x2) - StepVec_x,...
    (LAST.y1 - FIRST.y2) - StepVec_y,...
    (LAST.x0 - FIRST.x0) - StepVec_x,...
    (LAST.y0 - FIRST.y0) - StepVec_y,...
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