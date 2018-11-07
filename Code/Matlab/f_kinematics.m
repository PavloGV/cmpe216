% File: f_kinematics.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Brief: Given angles, find end-effector position

function end_position = f_kinematics(lengths, angles)
    x2 = lengths(1)*cos(angles(1)) + lengths(2)*cos(angles(1) + angles(2));
    y2 = lengths(1)*sin(angles(1)) + lengths(2)*sin(angles(1) + angles(2));
    end_position = [x2 y2];
end