function output = Endpoint_SingleTwo(input)

P_dyn = input.auxdata.dynamics;
P_cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

switch P_cost.method
    case 'CoT'
        
        %The objective function to be minimized is the cost of transport (CoT).
        %   CoT = Work / (Weight*Distance);
        work = input.phase(1).integral;
        weight = P_dyn.g*(P_dyn.m1 + P_dyn.m2 + P_dyn.M);
        verySmallNumber = 1e-6;   %(m) Prevent divide by zero errors
        
%%%% HACK %%%%
error('Need to fix HACK in calculation for CoT');
%Need to compute the distance travelled by the center of mass
startPos = input.phase(1).initialstate(1);
endPos  = input.phase(1).finalstate(1);
%%%% DONE %%%%

        
        distance = startPos - endPos;
        %May want to be able to walk backwards, so need absolute value.
        distance = SmoothAbs(distance, P_cost.smoothing.distance) + verySmallNumber;
        output.objective = work/(weight*distance); %Cost of Transport
        
    case 'Squared'
        output.objective = input.phase(1).integral;
    otherwise
        error('Invalid cost metric')
end

end