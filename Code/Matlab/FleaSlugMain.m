% File: FleaSlugMain.m
% Author: Pavlo Vlastos
% UC Santa Cruz, CMPE 216 Bio-Inspired Locomotion

% Distances are in meters
% Angles are in radians

% Setup
clear all;
close all;
clc;

thigh_pos = [0 0];
thigh_len = 0.02;
thigh_ang = pi/3;

shin_len = 0.03;
shin_an = pi/3;

thigh = Segment(thigh_pos, thigh_len, thigh_ang);