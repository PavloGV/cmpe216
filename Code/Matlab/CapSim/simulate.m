function [X,L,j_list] = simulate(E,S)

    % default values
    nm          = E.nm;                 % number of state variables
    D.x0        = [2*randn(nm,1);...    % initial states
                 0.1*randn(nm,1)]; 
    %%%%         
    D.x0(1) = -abs(E.walls(2,3));       % x position initial
    D.x0(2) = -abs(E.walls(1,3));       % y position initial
    D.x0(3) = 0.1;                      % body angle initial
    D.x0(4) = 0.3142;
    D.x0(5) = -3.1416;
    % object 4
    D.x0(6) = -2;                       % x location of step 1 of staircase
    D.x0(7) = -abs(E.walls(1,3));       % y location of step 1 of staircase
    D.x0(8) = 0;                        % angle initial
    % object 5
    D.x0(9) = 2;                        % x location of step 2 of staircase
    D.x0(10) = -abs(E.walls(1,3));      % y location of step 2 of staircase
    D.x0(11) = 0;                     	
    % object 6
    D.x0(12) = 6;                       % x location of step 3 of staircase
    D.x0(13) = -abs(E.walls(1,3));      % y location of step 3 of staircase
    D.x0(14) = 0;
    % object 7
    D.x0(15) = 2;                       % x location of step 4 of staircase
    D.x0(16) = -abs(E.walls(1,3))+3;    % y location of step 4 of staircase
    D.x0(17) = 0;
    % object 8
    D.x0(18) = 6;                       % x location of step 5 of staircase
    D.x0(19) = -abs(E.walls(1,3))+3;    % y location of step 5 of staircase
    D.x0(20) = 0;
    % object 9
    D.x0(21) = 6;                       % x location of step 5 of staircase
    D.x0(22) = -abs(E.walls(1,3))+5;    % y location of step 5 of staircase
    D.x0(23) = 0;
    
    %%%%
    D.N         = 10000;                % number of time steps
    D.policy    = @(x) x;               % policy (controller)
    D.graphs    = 0;                    % draw graphs ?
    D.frames    = 1;                    % draw frames (simulation) ?
    D.spring    = 20;                   % spring constant
    D.p_steps   = 5;                    % constraint projection steps
    D.c_points  = 0;                    % draw collision points?

    if nargin == 1
       S  = struct(); % empty structure - use defaults
    end

    S           = setOpts(D,S);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % --- initial state
    x           = S.x0(1:E.nm);
    v           = S.x0(E.nm+1:end);
    
    % --- constants
    n           = length(E.masses);

    % --- graphics
    G.fig = findobj(0,'name','CapSim');
    if  isempty(G.fig)
       G.fig = figure();
    end
    figure(G.fig);
    set(gcf, 'Position', [800 50 800 600]);
    clf;

    if 0 %OpenGl rarely works well
       rend = 'OpenGL';
    else
       rend = 'Painters';
    end
    set(G.fig,...
       'Color','white',...
       'Renderer', rend,...
       'MenuBar', 'figure',...
       'WindowButtonDownFcn', @fDown,...
       'WindowButtonUpFcn', @fUp,...
       'KeyPressFcn', @fKey,...
       'NumberTitle', 'off',...
       'Name', 'CapSim');
    colormap bone;

    if S.graphs
       G.ax1 = subplot(2,3,[1 2 4 5]);
       G.ax2  = subplot(2,3,3);
       set(G.ax2,'box', 'on'); 
       G.ax3  = subplot(2,3,6);
       set(G.ax3,'box', 'on'); 
    else
       G.ax1 = axes();
    end
    setappdata(G.fig,'Stop',0);

    % --- counters
    time_steps  = 0;
    i_time      = 0;
    X           = [];
    L           = [];
    It          = [];
    
    % ---- data buffers
    a_list = zeros(3,S.N);      % body angle    1, 2, 3, ... , N
                                % hip angle     1, 2, 3. ... , N
                                % knee angle    1, 2, 3, ... , N
    d_list = zeros(2,S.N);
    e_list = zeros(1,S.N);
    j_p_list = zeros(1,S.N);
    j_list = zeros(1,S.N);
    %%%% Impulse for Flea Jump
    % | x1     x2     ... xN     |
    % | y1     y2     ... yN     |
    % | angle1 angle2 ... angleN |
    %%%%
    
    %%%% Flags for gif and jump agilty recording
    gif_flag = 0;
    j_flag = 1;     % reduces number of time steps
    
    
    
    time_limit = S.N;
    
    % record jump agility
    if (j_flag == 1)
        time_limit = 300
    end
        
    % to make gif
    if (gif_flag == 1)
        f = G.fig;
        axis tight manual
        filename = 'testAnimated.gif'
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    while ~getappdata(G.fig,'Stop') && (time_steps <= time_limit)
        % xc is the center of masses positions for the capsules
        %  __         __
        % | m0x m1x m2x | x
        % | m0y m1y m2y | y
        % | th0 th1 th2 | angle
        %  --         --
        [xc,J]      = m2c(x,E); 
        theta_knee = abs(xc(3,2)-xc(3,3))-pi;
        theta_hip = abs(mod(xc(3,2), 2*pi));
        body_or = 0.1;
        
        % get spring force
        cp = getappdata(G.fig,'cursorPos');
        u  = zeros(3*n,1); % I think this is the input force or impulse 
        % in the case of the flea it is a 9 by 1 vector or three 3 by 1 
        % vectors for each capsulse (as there are only 3 capsules for the 
        % flea each 3 by 1 is as follows: [vx vy angle_rate]'
        if ~isempty(cp)
%             if isnan(w)
%                 % find nearest point on nearest capsule
%                 [w,rel] = nearest(xc, E.radii, cp);
%             end
%             xr             = [cos(xc(3,w)) -sin(xc(3,w)); sin(xc(3,w)) cos(xc(3,w))]*rel;
%             xw             = xc(1:2,w) + xr;
%             cursor         = [cp' xw];
%             xf             = cp'-xw;
%             u(3*w-2:3*w)   = D.spring*[xf; xf'*[0 -1;1 0]*xr];
            %%%%
            jump_mag = 40;
            if (theta_knee < pi) % Jumping allowed when knee is NOT fully extended
                u(6) = 2*jump_mag/3;
                u(9) = -jump_mag;
            end
            %%%%
        else
            %%%% if impulse train is uncommented above
            %%%%
            cursor         = [];
            w              = nan;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Flea Slug Brain
        % Have knee spring back
        k_angle = 10;
        if abs(theta_knee) > pi/8
            u(9) = u(9) + k_angle;
        end
        % Have hip spring back
        h_angle = 20;
        if theta_hip > pi/4 && theta_hip < pi/2
            u(6) = u(6) - h_angle;
        elseif theta_hip >= 3*pi/4
            u(6) = u(6) + h_angle/2;
        end
        
        % Have body stay level
        %         b_angle = 5;
        %         if xc(3,1) >= body_or
        %             u(3) = u(3) - b_angle;
        %         elseif xc(3,1) < body_or
        %             u(3) = u(3) + b_angle;
        %         end
        
        % Record angles
        a_list(:,time_steps+1) = [xc(3,1) theta_hip theta_knee]';
        
        % Record movement of center of mass
        d_list(:,time_steps+1) = [xc(1,1) xc(2,1)]';
        
        % Record Jump agility
        j_list(time_steps+1) =  max_height/(t_stance + t_apogee);
        % pause(0.1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % calculate info data
        energy = 0.5*full(v'*J'*E.M*J*v);
        
        %%%%
        % record energy
        e_list(time_steps+1) = energy;
        %%%%
        
        info  = sprintf('Kinetic Energy: %-6.3g Joules \nDynamics Calculation Time: %-4.1fms\nTotal Time Steps: %d',energy,1000*i_time,time_steps);


        % integrate (aka the UPDATE)
        tic;
        [x,v,lam,c_points]   = dynamics(x,v,u,E);
        
        % time measurement
        i_time               = toc;
        if time_steps < S.p_steps % move to right place
            v = zeros(size(v));
        end   

        % draw collision points?
        if ~S.c_points
            c_points = zeros(0,2);
        end

        % draw (previous) state
        drawFrame(G.ax1, E, xc, c_points, cursor, info);   

        % save traces
        X           = [X [x;v]];
        L           = [L lam];
        It          = [It i_time];

        if S.graphs
            if size(X,2) > 1
                tspan = max(1,size(X,2)-round(8/E.dt)):size(X,2);
                set(G.fig,'currentaxes',G.ax2);      
                plot(X(1:size(x,1),tspan)');
                set(G.ax2,'Xtick',[],'Ytick',[],'Xlim',[1 length(tspan)]);
                set(G.fig,'currentaxes',G.ax3);      
                plot(L(:,tspan)');
                % set(G.ax3,'Xtick',[],'Ytick',[],'Xlim',[1 length(tspan)]);
                set(G.ax3,'Xtick',[],'Xlim',[1 length(tspan)]);
                set(G.fig,'currentaxes',G.ax1);
            end
        end

        drawnow;
        time_steps = time_steps+1;
        
        % for gif
        if (gif_flag == 1)
            frame = getframe(f); 
            im = frame2im(frame); 
            [imind,cm] = rgb2ind(im,256); 

                  % Write to the GIF File 
              if time_steps == 1 
                  imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
              else 
                  imwrite(imind,cm,filename,'gif','WriteMode','append'); 
              end 
        end
    end
    
    %%%% Plot Results
    figure(2)
    hold on
    plot(1:1:time_steps, a_list(1,1:time_steps))
    plot(1:1:time_steps, a_list(2,1:time_steps))
    plot(1:1:time_steps, a_list(3,1:time_steps))
    legend('Body', 'Body & Femur', 'Femur and Tibia');
    ylabel('Angle (Radians)');
    xlabel('Time step');
    title('Leg Component Angles Over Time');
    hold off
    
    figure(3)
    hold on
    plot(d_list(1,1:time_steps), d_list(2,1:time_steps))
    ylabel('Y position');
    xlabel('X position');
    title('Position');
    hold off
    
    figure(4)
    hold on
    plot((1:1:time_steps), e_list(1:time_steps))
    ylabel('Joules');
    xlabel('Time Steps');
    title('System Kinetic Energy');
    hold off
    %%%%
    
    fprintf('Average dynamics time: %-4.1fms\n',1000*mean(It));

function [w, rel] = nearest(xc, radii, cp)
    n        = size(xc,2);
    xc       = [xc [cp'; 0]];
    radii    = [radii; 0 0];
    na       = n;
    [e1,e2]  = deal((1:n)',(n+1)*ones(n,1));

    z        = zeros(8,4); 
    z([1:2:8;2:2:8]+8*[0:3;0:3]) = 1;
    z        = z';

    r        = radii';                              % [2  n] radii
    x        = xc(1:2,:);                           % [2  n] center-of-mass positions
    a        = xc(3,:);                             % [1  n] angles           

    v        = [cos(a); sin(a)];                    % [2  n] unit vectors along capsule axes
    p        = [x+r([1 1],:).*v;  x-r([1 1],:).*v]; % [4  n] end points of segments [x1 y1 x2 y2]'

    pp       = [p(:,e1); p(:,e2)];                  % [8 na] segment endpoint pairs      [p1; p2]
    xx       = [x([1 2 1 2],e2); x([1 2 1 2],e1)];  % [8 na] target segment centers      [x2; x1] 
    vv       = [v([1 2 1 2],e2); v([1 2 1 2],e1)];  % [8 na] target segment unit vectors [v2; v1]
    rr1      = [r([1 1],e2); r([1 1],e1)];          % [4 na] first radii  [r1 r1 r2 r2]'
    rr2      = [r([2 2],e2); r([2 2],e1)];          % [4 na] second radii [r1 r1 r2 r2]'

    t        = z*((pp - xx).*vv);                   % [4 na] two offset scalars for each pair 
    t        = min(rr1, max(-rr1,t));               % [4 na] clamp the offset scalars to segment ends
    uu       = xx + vv.*t([1 1 2 2 3 3 4 4],:);     % [8 na] closest point candidates on target segments
    d        = z*((uu-pp).^2);                      % [4 na] distance to candidates
    [l,i]    = min(d,[],1);                         % find closest point-pair
    ax       = i   + 4*(0:na-1);                    % indexes to the sources
    bx       = 5-i + 4*(0:na-1);                    % indexes to the targets
    ix       = [2*i-1; 2*i]+[8;8]*(0:na-1);         % indexes to the closest pair sources
    jx       = [2*(5-i)-1; 2*(5-i)]+[8;8]*(0:na-1); % indexes to the closest pair targets
    s        = [pp(ix); uu(ix)];                    % [4 na] [sources; targets] on segments
    sr       = uu(ix) - pp(ix);                     % [2 na] from sources to targets (on segments)
    sr       = sr .* ([1 1]'*sum(sr.^2,1).^(-.5));  % [2 na] normalize the direction
    dir      = sign(sum(sr.*(xx(ix)-xx(jx)),1));    % [1 na] sign correction in case of penetration
    sr       = sr .* dir([1 1],:);                  % [2 na] apply the correction
    rr3      = [[1 1]'*rr2(bx); [1 1]'*rr2(ax)];    % [4 na] select the right radii
    sc       = s + [sr; -sr].*rr3;                  % [4 na] [sources; targets] on capsule surfaces
    scr      = sc([3 4],:) - sc([1 2],:);           % [2 na] from sources to targets (on capsule surface)
    k        = sum(scr.*sr,1)';                     % [na 1] project on direction to get distance

    rel      = sc - [xx(jx); xx(ix)];               % [4 na] from centers to [sources; targets]
    rel(:,i>2)= rel([3:4 1:2]',i>2);                % [4 na] reorder to [e1; e2]

    [dum,w]  = min(k);                              % closest capsule
    rel      = expm([0 1;-1 0]*a(w))*rel(1:2,w);    % rotate rel to egocentric coordinates



function fDown(src, evnt)
    axis  = findobj(src, 'tag', 'simulation');
    cp    = get(axis, 'CurrentPoint');  
    cp    = cp(1, 1:2);
    xlim  = get(axis, 'xlim');
    ylim  = get(axis, 'ylim');
    if cp(1) > xlim(1) && cp(1) < xlim(2) && cp(2) > ylim(1) && cp(2) < ylim(2)
       set(src, 'WindowButtonMotionFcn', @fMove);
       setappdata(src, 'cursorPos', cp);
    end

function fMove(src, evnt)
    h = findobj(src, 'tag', 'simulation');
    cp = get(h, 'CurrentPoint');  
    cp = cp(1, 1:2);
    setappdata(src, 'cursorPos', cp); 
   
function fUp(src, evnt)
    set(src, 'WindowButtonMotionFcn', '');
    setappdata(src, 'cursorPos', ''); 

function fKey(fig, evnt)
    if strcmp(evnt.Key, 'escape')
       setappdata(fig, 'Stop', 1);
    end
