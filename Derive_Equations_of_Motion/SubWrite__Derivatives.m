%SubWrite__Derivatives

Ndof = size(States,1)/2;
N = num2str(Ndof);
fprintf(fid,'dStates = zeros(size(States));\n');
fprintf(fid,['dStates(:,(1:' N ')) = '...
    'States(:,(1+' N '):(' N '+' N '));\n']);
for idxAcc=1:Ndof
    writeIndex = idxAcc + Ndof;
    writeName = ['d' States{writeIndex,1}];
    writeData = Dynamics.(writeName);
    vectorizedExpression = vectorize(writeData);
    fprintf(fid,['dStates(:,' num2str(writeIndex) ') = ' ...
        vectorizedExpression ';\n']);
end %idxAcc
fprintf(fid,'\n');