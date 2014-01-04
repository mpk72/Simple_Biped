function Write_getPower_SingleStance(input)

States = input.States;
Directory = input.Directory;
Power = input.Power;
Actuators = input.Actuators;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/getPower_single.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function Power = ' ...
    'getPower_single(States, Actuators)\n']);
fprintf(fid,['%%function Power = ' ...
    'getPower_single(States, Actuators)\n']);
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
%                       Write Power Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Power);
for idxPos=1:length(names)
    fprintf(fid,['Power.' names{idxPos} ' = '...
        vectorize(Power.(names{idxPos})) ';\n']);
end
fprintf(fid,'\n');



fprintf(fid,'end\n');
fclose(fid);


end