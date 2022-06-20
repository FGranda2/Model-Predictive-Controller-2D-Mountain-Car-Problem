% get_cost: Function constructing Hessian of cost function
%
% Inputs:
%       r:              Non-negative scalar penalizing input
%       Q:              Symmetric positive semidefinite matrix penalizing
%                       errors in state
%       n_lookahead:    Length of MPC prediction horizon
%
% Outputs:
%       S:              Hessian matrix of cost function
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 3
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.03.07, SZ]    first version
% [22.03.02, SZ]    second version

function [S] = get_cost(r, Q, n_lookahead)
    % cost function
    S = [];
    for i = 1:1:n_lookahead
        S = blkdiag(S, r);
    end

    for i = 1:1:n_lookahead
        S = blkdiag(S, Q);
    end
end