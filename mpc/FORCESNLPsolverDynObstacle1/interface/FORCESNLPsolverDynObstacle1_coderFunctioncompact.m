% FORCESNLPsolverDynObstacle1 : A fast customized optimization solver.
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
% [OUTPUTS] = FORCESNLPsolverDynObstacle1(INPUTS) solves an optimization problem where:
% Inputs:
% - xinit - matrix of size [6x1]
% - x0 - matrix of size [80x1]
% - all_parameters - matrix of size [208x1]
% Outputs:
% - outputs - column vector of length 80
function [outputs] = FORCESNLPsolverDynObstacle1(xinit, x0, all_parameters)
    
    [output, ~, ~] = FORCESNLPsolverDynObstacle1Buildable.forcesCall(xinit, x0, all_parameters);
    outputs = coder.nullcopy(zeros(80,1));
    outputs(1:10) = output.x1;
    outputs(11:20) = output.x2;
    outputs(21:30) = output.x3;
    outputs(31:40) = output.x4;
    outputs(41:50) = output.x5;
    outputs(51:60) = output.x6;
    outputs(61:70) = output.x7;
    outputs(71:80) = output.x8;
end
