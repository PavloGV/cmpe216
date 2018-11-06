% File: FleaSlugMain.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

% Setup
clear all;
close all;
clc;

hip0 = [0 0];

% Leg specifications
% Segment 1
flea_seg(1).length = 5; 
flea_seg(1).angle = 36.87*pi/180;
flea_seg(1).position = [0 0];

% Segment 2
flea_seg(2).length = 10; 
flea_seg(2).angle = 90*pi/180; 
flea_seg(2).position = [0 0];

seg_list(1) = flea_seg(1);
for i = 2:1:length(flea_seg)
    seg_list(end+1) = flea_seg(i);
end

leg_out = Leg(hip0, flea_seg);
leg_out.params(1)
leg_out.params(2)

% loops at 10 frames per second, or 1 frame every 100ms
dt = 0.1;       % which in "real" time translates to 100ms
t_count = 0.1;  % time counter
n = 100;        % number of iterations
figure(1)
hold on

for k = 1:1:n
    % update display
    clf;
    line(  [leg_out(1).params(1).joint_pos(1) ...
            leg_out(1).params(1).end_pos(1)], ...
            [leg_out(1).params(1).joint_pos(2) ...
            leg_out(1).params(1).end_pos(2)]);
    line(   [leg_out(1).params(2).joint_pos(1) ...
            leg_out(1).params(2).end_pos(1)], ...
            [leg_out(1).params(2).joint_pos(2) ...
            leg_out(1).params(2).end_pos(2)]);
    axis([-1 10 -1 10]);
    text(0,9,"time = "+num2str(t_count,'%2.1f')+"s",'FontSize',14);
    pause(dt)
    t_count = dt + t_count;
    
    % update position
    
    
end
    
    