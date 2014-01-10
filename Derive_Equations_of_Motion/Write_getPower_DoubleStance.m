function Write_getPower_DoubleStance(input)

States = input.States;
Directory = input.Directory;
Power = input.Power;
Actuators = input.Actuators;
Intermediate = input.Intermediate;
Kinematics = input.Kinematics;
Parameters = input.Parameters;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/getPower_double.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function Power = ' ...
    'getPower_double(States, Actuators, Parameters)\n']);
fprintf(fid,['%%function Power = ' ...
    'getPower_double(States, Actuators, Parameters)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_getPower_DoubleStance()\n']);
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
%                       Write Power Struct                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

names = fieldnames(Power);
fprintf(fid,'Power = zeros(size(Actuators));\n');
for idx=1:length(names)
    fprintf(fid,['Power(:,' num2str(idx) ') = '...
        vectorize(Power.(names{idx})) '; %%' names{idx} '\n']);
end
fprintf(fid,'\n');



fprintf(fid,'end\n');
fclose(fid);


end