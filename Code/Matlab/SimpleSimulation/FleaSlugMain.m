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
g = 9.81;       % gravity
precision = 5; 	% number of decimal places
dt = 0.010;   	% time it takes for one loop iteration in seconds
t_count = dt;	% time counter
n = 50;         % number of iterations
t_max = n*dt;
t_offset = 0;

t_push = 0.3;	% how long it takes to push off the ground, 0.25*period
n_push = n*(t_push/t_max);  % how many iterations to push off the ground
f_push= 1/(4*t_push);       % frequency of the push, 4*t_push for full period
time_steps = linspace(1,n,n);

angles_foot = pi/2*sin(linspace(0,pi/2,n_push));
angles_knee = pi*cos(linspace(0,pi/2,n_push));
angle_list = [angles_foot; angles_knee];

m_body = 0.007;	% mass of body in kilograms

l_tibia = 0.07;
l_femur = l_tibia;
seg_lengths = [l_tibia l_femur];

theta_push = pi/2.1;  % initial launch angle
max_leg_length = l_tibia+l_femur;
ep_dx = round(max_leg_length*cos(theta_push),precision);
ep_dy = round(max_leg_length*sin(theta_push),precision);
ep_rel_x = linspace(0,ep_dx,n_push);
ep_rel_y = linspace(0,ep_dy,n_push);
ep_pos = [ep_rel_x; ep_rel_y];

% number of loop iterations it takes to push off of the ground
push_gate_iterations = length(ep_pos); 

F_jump = 100*abs(m_body*g);
distance_push = max_leg_length/100;
work_energy = F_jump*distance_push;
x_spring = distance_push;
k_spring = (2*work_energy)/(x_spring^2);
v0 = sqrt((k_spring*x_spring^2)/m_body);
v0y = v0*sin(theta_push);
v0x = v0*cos(theta_push);

dx = v0x*dt;
dy = v0y*dt;

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

dyList = zeros(1,n);
yList = zeros(1,n);
t_rel_list = zeros(1,n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% loops at 10 frames per second, or 1 frame every 100ms
% Loop specific paramters
height = 0.3;
width = height;

% to make gif
f = getframe;
[im, map] = rgb2ind(f.cdata,256,'nodither');
im(1,1,1,20) = 0;

figure(1)
hold on
for k = time_steps
    clf;
    
    % format
    stair0 = line([-0.2*width, width], [0 0]);
    stair1 = line([0.35 0.55],[0.06243 0.06243]);
    stair2 = line([0.55 0.75],[0.06243+0.1 0.06243+0.1]);
    stair3 = line([0.75 0.95],[0.06243+0.2 0.06243+0.2]);
    stair4 = line([0.35 0.35],[0 0.06243]);
    stair5 = line([0.55 0.55],[0.06243 0.06243+0.1]);
    stair6 = line([0.75 0.75],[0.06243+0.1 0.06243+0.2]);
    
    line0 = line(leg(1).dx0, leg(1).dy0);   % leg
    line1 = line(leg(1).dx1, leg(1).dy1);   %
    line0.Marker = 'o';
    line1.Marker = 'o';
    axis([-0.2*width width -0.2*height height]);
    text(0,0.9*height,"time = "+num2str(t_count,'%2.2f')+"s",'FontSize',14);
    pause(dt)
    t_count = dt + t_count;
    
    % update position values
    % push
    if(k<(push_gate_iterations+1))
        leg = update(leg, seg_lengths, ep_pos(:,k), precision);
        t_offset = t_offset + dt;
    else
        t_rel = t_count - t_offset;
        t_rel_list(k) = t_rel;
        dy = v0y*t_rel - 0.5*g*t_rel^2;
        
        if (leg(1).dy0(1)+dy <= 0)
            dy = 0;
            dx = 0;
        end
        dyList(k) = dy;
        dy = 0;
        dx = 0;

        leg(1).dx0(1) = leg(1).dx0(1) + dx; % change in x dimension
        leg(1).dx0(2) = leg(1).dx0(2) + dx;
        leg(1).dx1(1) = leg(1).dx1(1) + dx;
        leg(1).dx1(2) = leg(1).dx1(2) + dx;

        leg(1).dy0(1) = leg(1).dy0(1) + dy; % change in y dimension
        leg(1).dy0(2) = leg(1).dy0(2) + dy;
        leg(1).dy1(1) = leg(1).dy1(1) + dy;
        leg(1).dy1(2) = leg(1).dy1(2) + dy;
        
        yList(k) = leg(1).dy0(1);
    end
    
    % for gif
    f = getframe;
    im(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
end

% finish gif
imwrite(im,map,'hop.gif','DelayTime',0,'LoopCount', inf)

figure(2)
hold on
plot(time_steps, dyList)
plot(time_steps, yList)
plot(time_steps, t_rel_list)
legend('dy','y','t_{rel}');
hold off

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
    