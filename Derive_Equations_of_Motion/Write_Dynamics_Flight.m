function Write_Dynamics_Flight(input)

States = input.States;
Dynamics = input.Dynamics;
Actuators = input.Actuators;
Parameters = input.Parameters;
Directory = input.Directory;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/dynamics_flight.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function dStates = ' ...
    'dynamics_flight(States, Actuators, Parameters)\n']);
fprintf(fid,['%%function dStates = ' ...
    'dynamics_flight(States, Actuators, Parameters)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Dynamics_Flight()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Parse Inputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

SubWrite__Parse_Inputs;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Accelerations                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

SubWrite__Derivatives

fprintf(fid,'end\n');
fclose(fid);


end