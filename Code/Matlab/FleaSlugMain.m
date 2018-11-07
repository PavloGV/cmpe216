% File: FleaSlugMain.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

% This simulation treats the hip of the leg as the end-effector of a robot
% arm and the origin/shoulder of the robot arm as the foot of the robot

% Setup
clear all;
close all;
clc;

% Parameters
dt = 0.1;           % which in "real" time translates to 100ms
t_count = 0.1;      % time counter
n = 100;            % number of iterations
t_max = n*dt;       
foot = [0 0];       % starting point of the foot of the robot
t_push = 0.5;       % how long it takes to push off the ground
n_push = n*(t_push/t_max);
time_steps = linspace(1,n,n);
angle_foot = pi/2*sin(linspace(0,1,n_push));
angle_knee = pi*cos(time_steps);

m_body = 0.007;
l_femur = 0.05;
l_tibia = 0.07;
leg = [l_tibia l_femur];

% Leg specifications
% % Segment 1
% flea_seg(1).length = 5; 
% flea_seg(1).angle = 36.87*pi/180;
% flea_seg(1).position = [0 0];
% 
% % Segment 2
% flea_seg(2).length = 10; 
% flea_seg(2).angle = 90*pi/180; 
% flea_seg(2).position = [0 0];
% 
% seg_list(1) = flea_seg(1);
% for i = 2:1:length(flea_seg)
%     seg_list(end+1) = flea_seg(i);
% end
% 
% leg_out = Leg(foot, flea_seg);
% leg_out.params(1)
% leg_out.params(2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% loops at 10 frames per second, or 1 frame every 100ms
figure(1)
hold on
for k = time_steps
    % update display
    clf;
%     line(  [leg_out(1).params(1).joint_pos(1) ...
%             leg_out(1).params(1).end_pos(1)], ...
%             [leg_out(1).params(1).joint_pos(2) ...
%             leg_out(1).params(1).end_pos(2)]);
%     line(   [leg_out(1).params(2).joint_pos(1) ...
%             leg_out(1).params(2).end_pos(1)], ...
%             [leg_out(1).params(2).joint_pos(2) ...
%             leg_out(1).params(2).end_pos(2)]);
    axis([-1 10 -1 10]);
    text(0,9,"time = "+num2str(t_count,'%2.1f')+"s",'FontSize',14);
    pause(dt)
    t_count = dt + t_count;
    
    % update position
    
end
    
    