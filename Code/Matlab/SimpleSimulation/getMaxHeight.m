% File: getMaxHeight.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Brief: 

function y_max = getMaxHeight(init_vel,t)
    g = 9.81;
    y_max = (init_vel^2)/(2*g);
end