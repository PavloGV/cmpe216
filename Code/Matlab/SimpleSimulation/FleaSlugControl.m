% File: FleaSlugControl.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Setup
clear all;
close all;
clc;

% System
m_body = 0.007;	% mass of body in kilograms
l_tibia = 0.07;
l_femur = l_tibia;
c = 0.1; % damper
g = 9.81;
F_jump = abs(m_body*g);
max_leg_length = l_tibia+l_femur;
distance_push = max_leg_length;
work_energy = F_jump*distance_push;
x_spring = distance_push;
k_spring = (2*work_energy)/(x_spring^2);

s = tf('s');

G = 1/(m_body*s^2 + c*s + k_spring)
k = 2;
sys  = feedback(G,k);

figure(1);
hold on
for i = 1:1:5
    c = 0.1*i; % damper
    G = 1/(m_body*s^2 + c*s + k_spring)
    sys  = feedback(G,k);
    step(sys)
end
grid on
hold off