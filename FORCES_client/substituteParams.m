% Substitution of parametric problem data
%
%   STAGES = SUBSTITUIONPARAMS(STAGES, PARAMSTRUCT, PARAM)
%   returns an array of multistage structs without parameters that can be
%   used by STAGES2QCQP to form the matrices of a (sparse) QCQP, which then
%   in turn can be solved by other solvers.
%
%       STAGES:         is the structure of the multistage formulation
%       PARAMSTRUCT:    are the parameters generated by NEWPARAM
%       PARAM:          are the values of the parameters
%
% See also MULTISTAGEPROBLEM, NEWPARAM, STAGES2QCQP
%   
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.
