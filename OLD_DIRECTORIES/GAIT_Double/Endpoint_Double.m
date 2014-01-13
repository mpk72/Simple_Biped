function output = Endpoint_Double(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Objective Function                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
output.objective = input.phase(1).integral;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Constraints                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
START = input.phase(1).initialstate;
END = input.phase(1).finalstate;
L = input.auxdata.goal.Step_Length;

% Make the hip do something reasonable
output.eventgroup(1).event = [...   %horizontal component
    START(1) - 0.6*L,...
    END(1) - 0.9*L,...
    ];
output.eventgroup(2).event = [...   %vertical component
    START(2),...
    END(2),...
    ];

end