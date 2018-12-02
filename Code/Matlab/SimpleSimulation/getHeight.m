% File: getHeight.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Brief: 

function y = getHeight(init_vel,t)
    g = 9.81;
    y = init_vel*t - 0.5*g*t^2;
end