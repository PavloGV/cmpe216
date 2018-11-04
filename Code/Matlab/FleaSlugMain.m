% File: FleaSlugMain.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

% Setup
clear all;
close all;
clc;

globel_refence_x = [1; 0];
globel_refence_y = [0; 1];

thigh_pos = [0; 0];
thigh_len = 5;
thigh_ang = 36.87*pi/180;

shin_len = 0.03;
shin_an = pi/3;

thigh = Segment(thigh_pos, thigh_ang, thigh_len, globel_refence_x);