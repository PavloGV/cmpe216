% Class: Seg
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

classdef Segment < handle
    properties
        joint_pos = [0 0];
        foot_pos = [1 1];
        
        len_x = norm([joint_pos(1) foot_pos(1)]);
        len_y = norm([joint_pos(2) foot_pos(2)]);
        len = sqrt(len_x^2 + len_y^2);
        
        ref_joint_pos = [0 0];
        ref_foot_pos = [1 0];
        
        angle = arcsin(len_y/len);
    end
    
    methods
        % Class constructor 
        function seg_obj = Seg(origin_pos, joint_pos, foot_pos):
            
        end
    end
end

