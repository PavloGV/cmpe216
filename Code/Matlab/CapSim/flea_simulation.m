% File: flea_simulation.m
% Author: Pavlo Vlastos
% Acknowledgements: This is a file I wrote for the CapSim Matlab
% simulation. This file is based off of the test_simulation.m from CapSim
% Credit for the CapSim physics engine goes to Yuval, at
% https://www.mathworks.com/matlabcentral/fileexchange/29249-capsim-the-matlab-physics-engine

close all
clear all 
clc

% simulate mass varying
test_N = 5;
jmp_ag_list = zeros(1,test_N);  % record jump agility 
jmp_fr_list = zeros(1,test_N);  % record jump frequency
jmp_time = zeros(1,test_N);
mass_list = zeros(1,test_N);

for i = 1:1:test_N
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
    if (i == 1)
        E.masses = E.masses/10;
    elseif (i == 2)
        E.masses = E.masses;        % the original
    elseif (i == 3)
        E.masses = E.masses*10;
    elseif (i == 4)
        E.masses = E.masses*100;
    elseif (i == 5)
        E.masses = E.masses*1000
    end
    mass_list(i) = sum(E.masses); % mass in grams.

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

    

    tic
    [X, L, mh, ts, ta,t_max,ts_max] = simulate(E);
    %ts = ts*t_max/ts_max;  % The time scaling is wrong for this method
    %ta = ts*t_max/ts_max;
    t_max = toc;            % New scaling method
    ts = ts*t_max/ts_max;
    ta = ta*t_max/ts_max;
    jmp_ag = mh/(ts+ta);
    jmp_ag_list(i) = jmp_ag;
    jmp_fr_list(i) = 1/(ts+ta);
    jmp_time(i) = ts+ta;
end

figure();
hold on
ylim([0 max(jmp_ag_list)]);
xlim([0 1]);
plot(jmp_time, jmp_ag, 'Marker','o');
m1str = sprintf('mass: %-4.2f grams',mass_list(1));
m2str = sprintf('mass: %-4.2f grams',mass_list(2));
m3str = sprintf('mass: %-4.2f grams',mass_list(3));
m4str = sprintf('mass: %-4.2f grams',mass_list(4));
m5str = sprintf('mass: %-4.2f grams',mass_list(5));
legend(m1str,m2str,m3str,m4str,m5str);
ylabel('Jump Agility (cm/s)');
xlabel('Stance Time + Apogee Time (ms)');
title('Varying Mass for Jump Agility');
hold off


