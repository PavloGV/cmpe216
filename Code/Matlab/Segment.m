% File: Segment.m
% Class: Seg
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

classdef Segment < handle
    properties
        len = -1;
        angle = pi/6;
        joint_pos = [0 0];

        len_x = len*cos(angle);
        len_y = len*sin(angle);

        end_pos = [len_x len_y];
        
        ref_joint_pos = [0 0];
        ref_foot_pos = [1 0];
        
    end
    
    methods
        % Class constructor 
        function seg_obj = Segment(origin_position, segment_length, segment_angle)
            if nargin > 0
                seg_obj.joint_pos = origin_position;
                seg_obj.len = segment_length;
                seg_obj.angle = segment_angle;
            else
                error('No arguments provided');
            end
        end
    end
end

