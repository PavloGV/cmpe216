% File: Segment.m
% Class: Seg
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

classdef Segment < handle
    properties
        length = -1;
        angle = pi/4;
        
        joint_pos = [0 0];
        end_pos = [0 0];
        
        ref_joint_pos = [0 0];
        ref_end_pos = [0 0];
    end
    methods
        % Class constructor 
        function seg_obj = Segment(origin_position, segment_angle, ...
                segment_length, reference_end)
            if nargin > 0
                
                % Set reference segment starting point
                % Set reference segment end point, should always be unit vector
                seg_obj.ref_joint_pos = origin_position;    
                seg_obj.ref_end_pos = reference_end;        
                
                % Rotate segment from the reference vector
                rot = [ cos(segment_angle) -sin(segment_angle); 
                        sin(segment_angle) cos(segment_angle)];
                rotated_segment = rot*(reference_end'-origin_position')...
                    + origin_position'; 
                
                % Set the segment object
                seg_obj.joint_pos = origin_position;
                seg_obj.angle = segment_angle;
                seg_obj.length = segment_length;
                seg_obj.end_pos = rotated_segment'*segment_length;
            else
                error('No arguments provided');
            end
        end
    end
end

