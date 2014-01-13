function Write_getPosVel_Flight(input)

States = input.States;
Position = input.Position;
Velocity = input.Velocity;
Directory = input.Directory;

CurrDir = cd;
if ~exist(Directory,'dir')
    mkdir(Directory);
end

fid = fopen([Directory '/getPosVel_flight.m'],'w');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Function Header                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

fprintf(fid,['function [Position, Velocity] = ' ...
    'getPosVel_flight(States)\n']);
fprintf(fid,['%%function [Position, Velocity] = ' ...
    'getPosVel_flight(States)\n']);
fprintf(fid,'%%\n%% Computer Generated File -- DO NOT EDIT \n');
fprintf(fid,['%%\n%% This function was created by the function '...
    'Write_getPosVel_Flight()\n']);
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

%Don't need Com properties:
Position = rmfield(Position,'CoM');

names = fieldnames(Position);
for idxPos=1:length(names)
    fprintf(fid,['Position.' names{idxPos} '.x = '...
        vectorize(Position.(names{idxPos})(1)) ';\n']);
    fprintf(fid,['Position.' names{idxPos} '.y = '...
        vectorize(Position.(names{idxPos})(2)) ';\n']);
end
fprintf(fid,'\n');

fprintf(fid,'if nargout==2\n');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Write Velocity Struct                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%Don't need Com properties:
Velocity = rmfield(Velocity,'CoM');

names = fieldnames(Velocity);
for idxPos=1:length(names)
    fprintf(fid,['    Velocity.' names{idxPos} '.x = '...
        vectorize(Velocity.(names{idxPos})(1)) ';\n']);
    fprintf(fid,['    Velocity.' names{idxPos} '.y = '...
        vectorize(Velocity.(names{idxPos})(2)) ';\n']);
end
fprintf(fid,'end\n');
fprintf(fid,'\n');


fprintf(fid,'end\n');
fclose(fid);


end