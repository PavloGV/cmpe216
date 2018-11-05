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
        params = [];
        origin = [-1 -1];
    end
    methods
        % Class constructor 
        function leg_obj = Leg(origin, bone_list)
            if nargin > 0
                leg_obj.origin = origin;
                for i = 2:1:length(bone_list)
                    % make a segment for each length specified in the list
                    leg_obj.params(end-1) = Segment(bone_list(i).origin, bone_list(i).angle, bone_list(i-1).length, 2*bone_list(i-1).end_pos);
                    % Segment(thigh_pos, thigh_ang, thigh_len, globel_refence_x);
                end
            else
                error('Not enough arguments provided');
            end
        end
    end
end