%SubWrite__Intermediate

N = size(Intermediate,1);
for idxInt=1:N
    writeName = Intermediate{idxInt,1};
    writeData = Kinematics.(writeName);
    vectorizedExpression = vectorize(writeData);
    fprintf(fid,[writeName ' = ' ...
        vectorizedExpression '; %%' Intermediate{idxInt,2} '\n']);
end %idxAcc
fprintf(fid,'\n');