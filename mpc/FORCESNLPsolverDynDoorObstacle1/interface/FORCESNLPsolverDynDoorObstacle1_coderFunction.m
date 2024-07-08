% FORCESNLPsolverDynDoorObstacle1 : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This software is intended for simulation and testing purposes only. 
% Use of this software for any commercial purpose is prohibited.
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = FORCESNLPsolverDynDoorObstacle1(INPUTS) solves an optimization problem where:
% Inputs:
% - xinit - matrix of size [6x1]
% - x0 - matrix of size [80x1]
% - all_parameters - matrix of size [288x1]
% Outputs:
% - x1 - column vector of length 10
% - x2 - column vector of length 10
% - x3 - column vector of length 10
% - x4 - column vector of length 10
% - x5 - column vector of length 10
% - x6 - column vector of length 10
% - x7 - column vector of length 10
% - x8 - column vector of length 10
function [x1, x2, x3, x4, x5, x6, x7, x8] = FORCESNLPsolverDynDoorObstacle1(xinit, x0, all_parameters)
    
    [output, ~, ~] = FORCESNLPsolverDynDoorObstacle1Buildable.forcesCall(xinit, x0, all_parameters);
    x1 = output.x1;
    x2 = output.x2;
    x3 = output.x3;
    x4 = output.x4;
    x5 = output.x5;
    x6 = output.x6;
    x7 = output.x7;
    x8 = output.x8;
end
