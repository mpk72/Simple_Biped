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

for idxState=1:size(States,1)
    fprintf(fid,[States{idxState,1} ' = States(:,' num2str(idxState) '); %% ' ...
        States{idxState,2} '\n']);
end %idxState
fprintf(fid,'\n');

for idxParam=1:size(Parameters,1)
    fprintf(fid,[Parameters{idxParam,1} ' = Parameters.' ...
        Parameters{idxParam,1} '; %% ' ...
        Parameters{idxParam,2} '\n']);
end %idxState
fprintf(fid,'\n');

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

fprintf(fid,'if nargout > 1 \n');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Power Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

for idxAct=1:size(Actuators,1)
    fprintf(fid,['    ' Actuators{idxAct,1} ' = Actuators(:,' num2str(idxAct) '); %% ' ...
        Actuators{idxAct,2} '\n']);
end %idxAct
fprintf(fid,'\n');

names = fieldnames(Power);
for idxPos=1:length(names)
    fprintf(fid,['    Power.' names{idxPos} ' = '...
        vectorize(Power.(names{idxPos})) ';\n']);
end
fprintf(fid,'\n');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Energy Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['    Energy.Potential = '...
    vectorize(Energy.Potential) ';\n']);
fprintf(fid,['    Energy.Kinetic = '...
    vectorize(Energy.Kinetic) ';\n']);
fprintf(fid,'    Energy.Total = Energy.Potential + Energy.Kinetic;\n');

fprintf(fid,'end\n\n');

fprintf(fid,'end\n');
fclose(fid);


end