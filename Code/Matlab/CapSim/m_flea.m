% File: m_flea
% Author: Pavlo Vlastos
% Acknowledgements: This file is made for the CapSim Matlab simulation that
%   I did NOT write. Credit for the simulation engine goes to Yuval, at
%   https://www.mathworks.com/matlabcentral/fileexchange/29249-capsim-the-matlab-physics-engine

function E = m_flea()

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
E.masses = [0.5 0.05 0.05]';

% Joint connections
E.joints = [1 2; 2 3];

% Joint locations
E.j_locs = [-1 1; -5 1; -1 1];
                        
% Joint angle limits - the flea will have none for now
E.j_constr  = [1 2]'; % Which joints have angle limits ?
E.a_lims    = [-pi/2 pi/2; -pi/2 pi/2]; % What are the angle limits

% Drawing order of capsules
E.draw_order = 1:3;

% Walls
E.walls       = [0 1 -5; 1 0 -7; -1 0 -7; 0 -1 -5];





