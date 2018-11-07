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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
precision = 5;      % number of decimal places
dt = 0.020;        	% which in "real" time translates to 20ms
t_count = dt;       % time counter
n = 200;            % number of iterations
t_max = n*dt;

t_push = t_max;     % how long it takes to push off the ground, 0.25*period
n_push = n*(t_push/t_max);  % how many iterations to push off the ground
f_push= 1/(4*t_push);       % frequency of the push, 4*t_push for full period
time_steps = linspace(1,n,n);

angles_foot = pi/2*sin(linspace(0,pi/2,n_push));
angles_knee = pi*cos(linspace(0,pi/2,n_push));
angle_list = [angles_foot; angles_knee];

m_body = 0.007;     % mass of body in kilograms

l_tibia = 0.07;
l_femur = l_tibia;
seg_lengths = [l_tibia l_femur];

theta_push = pi/3;  % initial launch angle
ep_dx = round((l_tibia+l_femur)*cos(theta_push),precision);
ep_dy = round((l_tibia+l_femur)*sin(theta_push),precision);
ep_rel_x = linspace(0,ep_dx,n_push);
ep_rel_y = linspace(0,ep_dy,n_push);
ep_pos = [ep_rel_x; ep_rel_y];

theta0 = 0;         % angle between ground and tibia
theta1 = pi;        % angle between ref line of tibia and femur

p0x = 0;                    % origin of robot foot
p0y = 0;
p1x = l_tibia*cos(theta0);  % knee coordinate
p1y = l_tibia*sin(theta0);

% hip coordinate or end-effector
p2 = f_kinematics(	[l_tibia l_femur], [theta0 theta1]);    
p2x = round(p2(1));
p2y = round(p2(2));

% leg struct
leg(1).dx0 = [p0x p1x];
leg(1).dy0 = [p0y p1y];
leg(1).dx1 = [p1x p2x];
leg(1).dy1 = [p1y p2y];
leg(1).theta0 = theta0;
leg(1).theta1 = theta1;

% Loop specific paramters
height = 0.5;
width = height;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% loops at 10 frames per second, or 1 frame every 100ms
figure(1)
hold on
for k = time_steps
    clf;
    
    % format
    
    
    line0 = line(leg(1).dx0, leg(1).dy0);   % leg
    line1 = line(leg(1).dx1, leg(1).dy1);   %
    line0.Marker = 'o';
    line1.Marker = 'o';
    axis([-0.1*width width -0.1*height height]);
    text(0,0.9*height,"time = "+num2str(t_count,'%2.1f')+"s",'FontSize',14);
    pause(dt)
    t_count = dt + t_count;
    
    % update position values
 
    leg = update(leg, seg_lengths, ep_pos(:,k), precision);

end

% helper functions
function leg_out = update(ap, seg_lens, ep_pos, precision)
    p0x = 0;	% x component of origin of robot foot
    p0y = 0;    % y component of origin of robot foot
    
    new_angles = inv_kinematics(seg_lens, ep_pos);
    
    p1x = seg_lens(1)*cos(new_angles(1));       % knee coordinate
    p1y = seg_lens(1)*sin(new_angles(1));
    
    % hip coordinate or end-effector
    p2 = f_kinematics(seg_lens, new_angles);	
    p2x = round(p2(1),precision);
    p2y = round(p2(2),precision);
 
    ap(1).dx0 = [p0x p1x];
    ap(1).dy0 = [p0y p1y];
    ap(1).dx1 = [p1x p2x];
    ap(1).dy1 = [p1y p2y];
    ap(1).theta0 = new_angles(1);
    ap(1).theta1 = new_angles(2);
    leg_out = ap;
end
    