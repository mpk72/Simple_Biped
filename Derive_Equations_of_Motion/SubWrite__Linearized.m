%SubWrite__Linearized

%Initialize the matricies:
[Nx,Nu] = size(B);
fprintf(fid,'Nt = size(States,1);\n');
fprintf(fid,['A = zeros(' num2str(Nx) ',' num2str(Nx) ',Nt);\n']);
fprintf(fid,['B = zeros(' num2str(Nx) ',' num2str(Nu) ',Nt);\n']);
fprintf(fid,'\n');

%Populate A matrix
for i=1:Nx
    for j=1:Nx
        writeData = vectorize(A(i,j));
        if ~strcmp(writeData,'0')
            if strcmp(writeData,'1')
                fprintf(fid,['A(' num2str(i) ',' num2str(j) ',:) = ones(1,1,Nt);\n']); 
            else
                fprintf(fid,['A(' num2str(i) ',' num2str(j) ',:) = ' writeData ';\n']); 
            end
        end
    end
end
fprintf(fid,'\n');

%Populate B matrix
for i=1:Nx
    for j=1:Nu
        writeData = vectorize(B(i,j));
        if ~strcmp(writeData,'0')
            if strcmp(writeData,'1')
                fprintf(fid,['B(' num2str(i) ',' num2str(j) ',:) = ones(1,1,Nt);\n']); 
            else
                fprintf(fid,['B(' num2str(i) ',' num2str(j) ',:) = ' writeData ';\n']); 
            end
        end
    end
end
fprintf(fid,'\n');