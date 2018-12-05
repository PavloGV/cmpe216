% File: flea_simulation.m
% Author: Pavlo Vlastos
% Acknowledgements: This is a file I wrote for the CapSim Matlab
% simulation. This file is based off of the test_simulation.m from CapSim
% Credit for the CapSim physics engine goes to Yuval, at
% https://www.mathworks.com/matlabcentral/fileexchange/29249-capsim-the-matlab-physics-engine

close all
clear all 
clc

% Define mechanism, a group of capsules connected by joints
% The following mechanism has three capsules
E.mechanisms = [1 1 1]';  

% Mechansisms can be free floating or anchored, this mechanism will be
% floating
E.anchors = [nan nan nan];

% No joint limits on this mechanism
E.a_constr = [];

% Define shape of capsules
E.radii = 0.1*  [0  4;...   % 1 Body
                 4  1;...   % 2 Femur
                 4  1];     % 3 Tibia

% Mass
E.masses = [0.5 0.5 0.5]';

% Joint connections
% capsule 1 is connected to 2 and 2 is connected to 3
E.joints = [1 2; 2 3]; 

% Joint locations
E.j_locs = [0 1; -1 1; -1 1];
                        
% Joint angle limits - the flea will have none for now
E.j_constr  = [2]'; % Which joints have angle limits ?
E.a_lims    = [-9*pi/10 9*pi/10]; % What are the angle limits

% Colissions
% No colissions between flea capsules
E.collidable = false(3,3);

% Drawing order of capsules
E.draw_order = 1:3;

% Walls
f_width = 10;
f_height = 7;
E.walls = [0 1 -f_height; 1 0 -f_width; -1 0 -f_width; 0 -1 -f_height];

% Add a "pebble" or capsule staricase 
ns = 6;	% number of blocks for steps, hence ns
n = length(E.mechanisms);
E.mechanisms  = [E.mechanisms; [2:ns+1]'];
E.anchors     = [E.anchors; nan(ns,n)];
E.radii       = [E.radii; ones(ns,2)];
E.masses      = pi*E.radii(:,2).^2 + 4*E.radii(:,1).*E.radii(:,2);
E.draw_order  = [E.draw_order (n+1:n+ns)];
n             = n+ns; 
E.collidable  = false(n,n);
E.collidable(1,[4 5 6 7 8 9]) = true;
E.collidable(2,[4 5 6 7 8 9]) = true;
E.collidable(3,[4 5 6 7 8 9]) = true;
E.collidable(5, 7)  = true;
E.collidable(6, 8)  = true;
E.collidable(8, 9)  = true;

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