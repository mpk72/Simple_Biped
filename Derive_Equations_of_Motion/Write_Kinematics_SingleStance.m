function Write_Kinematics_SingleStance(input)

States = input.States;
Energy = input.Energy;
Position = input.Position;
Velocity = input.Velocity;
Parameters = input.Parameters;
Directory = input.Directory;
Power = input.Power;
Actuators = input.Actuators;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/kinematics_single.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function [Position, Velocity, Power, Energy] = ' ...
    'kinematics_single(States, Actuators, Parameters)\n']);
fprintf(fid,['%%function [Position, Velocity, Power, Energy] = ' ...
    'kinematics_single(States, Actuators, Parameters)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_Kinematics_SingleStance()\n']);
fprintf(fid,['%% ' datestr(now) '\n%%\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% Matthew Kelly \n%% Cornell University \n%% \n\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Parse Inputs                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

SubWrite__Parse_Inputs;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Position Struct                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Position);
for idxPos=1:length(names)
    fprintf(fid,['Position.' names{idxPos} '.x = '...
        vectorize(Position.(names{idxPos})(1)) ';\n']);
    fprintf(fid,['Position.' names{idxPos} '.y = '...
        vectorize(Position.(names{idxPos})(2)) ';\n']);
end
fprintf(fid,'\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Velocity Struct                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Velocity);
for idxPos=1:length(names)
    fprintf(fid,['Velocity.' names{idxPos} '.x = '...
        vectorize(Velocity.(names{idxPos})(1)) ';\n']);
    fprintf(fid,['Velocity.' names{idxPos} '.y = '...
        vectorize(Velocity.(names{idxPos})(2)) ';\n']);
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