function Soln = jacobSolve(Equations,Unknowns)
%
% FUNCTION:
%   Solve a nonlinear system of equations by assuming that it is linear in
%   the unknown variables (which is true for these mechanics problems)
%
% ARGUMENTS:
%   Equations = [Nx1] vector of symbolic expressions that are equal to zero
%   Unknowns = [Nx1] vector of symbolic variables to solve Equations for
%
% RETURNS:
%   Soln = a struct with a field for each unknown
%
% The matlab solve command seems to have a problem with solving large
% systems of non-linear equations. In the case of classical mechanics
% problems, it turns out that these systems are not too hard to solve
% because they are actually linear in the accelerations and constraint
% forces. Assuming that this is true, then you can transform the equations
% into a linear system by taking partial derivatives. Once this step is
% done, then matlab does a great job of solving the linear system.
%
% MATH:
%   Equations = 0;                  % By Definition
%   Equations = A*x + b;            % Assume: form, A independant* of x
%   A = jacobian(Equations wrt x);  %
%   b = Equations - A*x;            %
%   0 = A*x + b;                    %
%   x = -A\b;                       % Solved!
%

A = simplify(jacobian(Equations,Unknowns));
b = simplify(Equations - A*Unknowns);
x = simplify(-A\b);

for i=1:length(Unknowns)
    Soln.(char(Unknowns(i))) = x(i);
end

end