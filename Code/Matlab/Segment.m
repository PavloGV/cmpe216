% File: Segment.m
% Class: Seg
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

classdef Segment < handle
    properties
        len = -1;
        angle = pi/4;
        
        joint_pos = [0 0];
        end_pos = [0 0];
        
        ref_joint_pos = [0 0];
        ref_end_pos = [1 0];
        
    end
    
    methods
        % Class constructor 
        function seg_obj = Segment(origin_position, segment_angle, segment_length, reference_end)
            if nargin > 0
                seg_obj.ref_end_pos = reference_end;    % Set reference segment end
                
                seg_obj.joint_pos = origin_position;
                seg_obj.angle = segment_angle;
                seg_obj.len = segment_length;
                
                len_x = seg_obj.len*cos(segment_angle); 
                len_y = seg_obj.len*sin(segment_angle);
                
                seg_obj.end_pos = [len_x len_y];        % Set end position of segment
                
                seg_obj.ref_joint_pos = seg_obj.joint_pos;
                
                
            else
                error('No arguments provided');
            end
        end
    end
end

