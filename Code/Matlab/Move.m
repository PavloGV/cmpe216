% File: Move.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Brief: This library has functions to manipulate vectors or segments of
% the flea leg.

function new_seg = rotate_segment(old_seg, angle)
    rot = [ cos(angle) -sin(angle); 
            sin(angle) cos(angle)];
    new_seg = rot*old_seg
end