% File: inv_kinematics.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Brief: Given end-effector position, find the angles

function angles = inv_kinematics(lengths, end_position)
    x2 = end_position(1);
    y2 = end_position(2);
    a1 = lengths(1);
    a2 = lengths(2);
    D = (x2^2 + y2^2 - a1^2 - a2^2)/(2*a1*a2);
    theta2 = acos(D);
    theta1 = atan(y2/x2)-atan((a2*sin(theta2))/(a1+a2*cos(theta2)));
    angles = [theta1 theta2];
end