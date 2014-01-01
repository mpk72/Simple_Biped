%SubWrite__ContactForces

for idxContact=1:size(Contacts,1)
    fprintf(fid,['%% contactForces(:,' num2str(idxContact) ') == '...
        Contacts{idxContact,1} ' == ' Contacts{idxContact,2} '\n']);
end %idxContact
fprintf(fid,['contactForces = zeros(size(States,1),' num2str(size(Contacts,1)) ');\n']);
for idxContact=1:size(Contacts,1)
    writeIndex = num2str(idxContact);
    writeName = Contacts{idxContact,1};
    writeData = Dynamics.(writeName);
    vectorizedExpression = vectorize(writeData);
    fprintf(fid,['contactForces(:,' writeIndex ') = ' ...
        vectorizedExpression ';\n']);
end %idxContact
fprintf(fid,'\n');