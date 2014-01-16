%SubWrite__Parse_Inputs

for idxState=1:size(States,1)
    fprintf(fid,[States{idxState,1} ' = States(:,' num2str(idxState) '); %% ' ...
        States{idxState,2} '\n']);
end %idxState
fprintf(fid,'\n');

if exist('Actuators','var') 
    for idxAct=1:size(Actuators,1)
        fprintf(fid,[Actuators{idxAct,1} ' = Actuators(:,' num2str(idxAct) '); %% ' ...
            Actuators{idxAct,2} '\n']);
    end %idxAct
    fprintf(fid,'\n');
end

if exist('Parameters','var')
    for idxParam=1:size(Parameters,1)
        fprintf(fid,[Parameters{idxParam,1} ' = Parameters.' ...
            Parameters{idxParam,1} '; %% ' ...
            Parameters{idxParam,2} '\n']);
    end %idxState
    fprintf(fid,'\n');
end