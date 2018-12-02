% File: Leg.m
% Class: Leg
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Notes:
% Distances are in meters
% Angles are in radians
% It is assumed that the list of segments comprising the leg are in order
% from the orgin to the "foot"
% Leg params is matrix of lengths, angles, and reference vectors, of each 
% part of the leg

classdef Leg < handle
    properties
        params;
        origin = [-1 -1];
    end
    methods
        % Class constructor 
        function leg_obj = Leg(first_joint, bone_list)
            if nargin > 0
                    
                if isempty(bone_list) == 0
                    leg_obj.origin = first_joint;
                    leg_obj(end).params = Segment(first_joint, ...
                        bone_list(1).angle, bone_list(1).length, [1 0]);
                    
                    if length(bone_list) > 1
                        for i = 2:1:length(bone_list)
                            % Generate reference segments for additional joints
                            temp_ref = [cos(bone_list(i-1).angle)...
                                sin(bone_list(i-1).angle)]' + ...
                                leg_obj.params(end).end_pos';
                            
                            leg_obj.params(end+1) = Segment(...
                                leg_obj.params(end).end_pos, ...
                                bone_list(i).angle, ...
                                bone_list(i).length,...
                                temp_ref');
                        end
                    end
                end
            else
                error('Not enough arguments provided');
            end
        end
    end
end