function Write_Kinematics_SingleStance(input)

States = input.States;
Energy = input.Energy;
Parameters = input.Parameters;
Directory = input.Directory;
Power = input.Power;
Actuators = input.Actuators;
Kinematics = input.Kinematics;
Intermediate = input.Intermediate;

if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/kinematics_double.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function [Kinematics, Power, Energy] = ' ...
    'kinematics_double(States, Actuators, Parameters)\n']);
fprintf(fid,['%%function [Kinematics, Power, Energy] = ' ...
    'kinematics_double(Kinematics, Parameters)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Kinematics_DoubleStance()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Parse Inputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

SubWrite__Parse_Inputs;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Intermediate Steps                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

SubWrite__Intermediate;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Position Struct                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Kinematics);
for idxPos=1:length(names)
    fprintf(fid,['Kinematics.' names{idxPos} ' = '...
        names{idxPos} ';\n']);
end
fprintf(fid,'\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Power Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Power);
for idxPos=1:length(names)
    fprintf(fid,['Power.' names{idxPos} ' = '...
        vectorize(Power.(names{idxPos})) ';\n']);
end
fprintf(fid,'\n');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Energy Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['Energy.Potential = '...
    vectorize(Energy.Potential) ';\n']);
fprintf(fid,['Energy.Kinetic = '...
    vectorize(Energy.Kinetic) ';\n']);
fprintf(fid,'Energy.Total = Energy.Potential + Energy.Kinetic;\n');

fprintf(fid,'\n');



fprintf(fid,'end\n');
fclose(fid);


end