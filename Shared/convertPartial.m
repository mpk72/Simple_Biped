function dataOut = convertPartial(dataIn,Phase)

%This function converts between a struct of named column vectors and a
%matrix.

switch Phase
    case 'D'
        dataOut = convert_D(dataIn);
    case 'S1'
        dataOut = convert_S1(dataIn);
    case 'S2'
        dataOut = convert_S2(dataIn);
    case 'F'
        dataOut = convert(dataIn);
    otherwise
        error('Unregognized phase!');
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Double Stance                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
function dataOut = convert_D(dataIn)
if isstruct(dataIn)
    dataOut = [...
        dataIn.x0,...
        dataIn.y0,...
        dataIn.dx0,...
        dataIn.dy0,...
        ];
else
    dataOut.x0 = dataIn(:,1);
    dataOut.y0 = dataIn(:,2);
    dataOut.dx0 = dataIn(:,3);
    dataOut.dy0 = dataIn(:,4);
end
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Single Stance One                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
function dataOut = convert_S1(dataIn)
if isstruct(dataIn)
    if isfield(dataIn,'x0')  %Then must be states
        dataOut = [...
            dataIn.x0,...
            dataIn.y0,...
            dataIn.x2,...
            dataIn.y2,...
            dataIn.dx0,...
            dataIn.dy0,...
            dataIn.dx2,...
            dataIn.dy2,...
            ];
    elseif isfield(dataIn,'F1')  %Then must be actuators
        dataOut = [...
            dataIn.F1,...
            dataIn.F2,...
            dataIn.T1,...
            dataIn.Thip,...
            ];
    else %Unrecognized input
        error('Unrecognized data for S1')
    end
else
    switch size(dataIn,2)
        case 8 %States
            dataOut.x0 = dataIn(:,1);
            dataOut.y0 = dataIn(:,2);
            dataOut.x2 = dataIn(:,3);
            dataOut.y2 = dataIn(:,4);
            dataOut.dx0 = dataIn(:,5);
            dataOut.dy0 = dataIn(:,6);
            dataOut.dx2 = dataIn(:,7);
            dataOut.dy2 = dataIn(:,8);
        case 4 %Actuators
            dataOut.F1 = dataIn(:,1);
            dataOut.F2 = dataIn(:,2);
            dataOut.T1 = dataIn(:,3);
            dataOut.Thip = dataIn(:,4);
        otherwise
            error('Unrecognized data for S1')
    end
end

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Single Stance Two                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
function dataOut = convert_S2(dataIn)
if isstruct(dataIn)
    if isfield(dataIn,'x0')  %Then must be states
        dataOut = [...
            dataIn.x0,...
            dataIn.y0,...
            dataIn.x1,...
            dataIn.y1,...
            dataIn.dx0,...
            dataIn.dy0,...
            dataIn.dx1,...
            dataIn.dy1,...
            ];
    elseif isfield(dataIn,'F1')  %Then must be actuators
        dataOut = [...
            dataIn.F1,...
            dataIn.F2,...
            dataIn.T2,...
            dataIn.Thip,...
            ];
    else %Unrecognized input
        error('Unrecognized data for S1')
    end
else
    switch size(dataIn,2)
        case 8 %States
            dataOut.x0 = dataIn(:,1);
            dataOut.y0 = dataIn(:,2);
            dataOut.x1 = dataIn(:,3);
            dataOut.y1 = dataIn(:,4);
            dataOut.dx0 = dataIn(:,5);
            dataOut.dy0 = dataIn(:,6);
            dataOut.dx1 = dataIn(:,7);
            dataOut.dy1 = dataIn(:,8);
        case 4 %Actuators
            dataOut.F1 = dataIn(:,1);
            dataOut.F2 = dataIn(:,2);
            dataOut.T2 = dataIn(:,3);
            dataOut.Thip = dataIn(:,4);
        otherwise
            error('Unrecognized data for S2')
    end
end

end



