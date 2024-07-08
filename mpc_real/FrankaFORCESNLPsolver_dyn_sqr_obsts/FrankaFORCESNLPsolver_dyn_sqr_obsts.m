% FrankaFORCESNLPsolver_dyn_sqr_obsts - a fast solver generated by FORCESPRO v6.0.0
%
%   OUTPUT = FrankaFORCESNLPsolver_dyn_sqr_obsts(PARAMS) solves a multistage problem
%   subject to the parameters supplied in the following struct:
%       PARAMS.x0 - column vector of length 70
%       PARAMS.xinit - column vector of length 3
%       PARAMS.all_parameters - column vector of length 180
%
%   OUTPUT returns the values of the last iteration of the solver where
%       OUTPUT.x01 - column vector of size 7
%       OUTPUT.x02 - column vector of size 7
%       OUTPUT.x03 - column vector of size 7
%       OUTPUT.x04 - column vector of size 7
%       OUTPUT.x05 - column vector of size 7
%       OUTPUT.x06 - column vector of size 7
%       OUTPUT.x07 - column vector of size 7
%       OUTPUT.x08 - column vector of size 7
%       OUTPUT.x09 - column vector of size 7
%       OUTPUT.x10 - column vector of size 7
%
%   [OUTPUT, EXITFLAG] = FrankaFORCESNLPsolver_dyn_sqr_obsts(PARAMS) returns additionally
%   the integer EXITFLAG indicating the state of the solution with 
%       1 - OPTIMAL solution has been found (subject to desired accuracy)
%       0 - Timeout - maximum number of iterations reached
%      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
%      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
%     -98 - Thread error
%     -99 - Locking mechanism error
%    -100 - License error
%    -101 - Insufficient number of internal memory instances
%    -102 - Number of threads larger than specified
%
%   [OUTPUT, EXITFLAG, INFO] = FrankaFORCESNLPsolver_dyn_sqr_obsts(PARAMS) returns 
%   additional information about the last iterate:
%       INFO.it         - number of iterations that lead to this result
%       INFO.res_eq     - max. equality constraint residual
%       INFO.res_ineq   - max. inequality constraint residual
%       INFO.rsnorm     - norm of stationarity condition
%       INFO.rcompnorm  - max of all complementarity violations
%       INFO.pobj       - primal objective
%       INFO.mu         - duality measure
%       INFO.solvetime  - time needed for solve (wall clock time)
%       INFO.fevalstime - time needed for function evaluations (wall clock time)
%       INFO.solver_id  - solver ID of FORCESPRO solver
%
% See also COPYING
