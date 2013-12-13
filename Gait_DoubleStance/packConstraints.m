function C = packConstraints(Cst,Phase)

%This function takes a struct of constraints (Cst) and packs it into a matrix to
%be exported to GPOPS. The constraints are different for each phase of
%motion and for the discrete events. 

switch Phase
    case 'D'
        C = [...
            Cst.footOneContactAngle,...  %=atan2(H1,V1);
            Cst.footTwoContactAngle,...  %=atan2(H2,V2);
            ];
    otherwise
        error('Invalid mode string!')
end

end