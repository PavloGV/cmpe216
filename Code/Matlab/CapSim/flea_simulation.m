% File: flea_simulation.m
% Author: Pavlo Vlastos
% Acknowledgements: This is a file I wrote for the CapSim Matlab
% simulation. This file is based off of the test_simulation.m from CapSim
% Credit for the CapSim physics engine goes to Yuval, at
% https://www.mathworks.com/matlabcentral/fileexchange/29249-capsim-the-matlab-physics-engine

close all
clear all
clc

% --- select model
E  = m_flea;

% --- global parameters
E.skin            = 2;
E.stretch         = 2;
E.k_fric          = 0.3;
E.RK              = 0;
E.friction_model  = 1;
E.solver          = 1;
E.k_drag          = .1;
E.margin          = 1;
E.walls(:,3)      = E.walls(:,3)-1e-5;
E.k_grav          = [0 -9.8 0];

% --- initialize
E                 = initE(E);

% simulate
simulate(E);