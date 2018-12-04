function [X,L] = simulate(E,S)

    % default values
    nm          = E.nm;                 % number of state variables
    D.x0        = [2*randn(nm,1);...    % initial states
                 0.1*randn(nm,1)];   
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
    %x           = S.x0(1:E.nm)
    v           = S.x0(E.nm+1:end);
    
    %   [ x  y  ? ? ?]
    x = [-2.9874 -4.7455 8.3968 0.3142 -3.1416]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % --- constants
    n           = length(E.masses);

    % --- graphics
    G.fig = findobj(0,'name','CapSim');
    if  isempty(G.fig)
       G.fig = figure();
    end
    figure(G.fig);
    set(gcf, 'Position', [100 100 800 600]);
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
    
    %%%% Impulse for Flea Jump
    % | x1     x2     ... xN     |
    % | y1     y2     ... yN     |
    % | angle1 angle2 ... angleN |
    l_angle = 90*pi/180;            % angle of launch in radians
    i_mag = 400;                    % impulse magnitude
    i_mag_x = i_mag*cos(l_angle);
    i_mag_y = i_mag*sin(l_angle);
    i_train = zeros(3, S.N);
%     i_train(:,S.N/16 - 2) = [i_mag_x i_mag_y 0]';
%     i_train(:,S.N/16 - 1) = [i_mag_x i_mag_y 0]';
%     i_train(:,S.N/16) = [i_mag_x i_mag_y 0]';
%     i_train(:,S.N/16 + 1) = [i_mag_x i_mag_y 0]';
%     i_train(:,S.N/16 + 2) = [i_mag_x i_mag_y 0]';
    %%%% Joint angle change for Flea Jump
    ang_train = zeros(1, S.N);
    theta = 0;
    ang_train(S.N/4 - 2) = theta;
    ang_train(S.N/4- 1) = theta;
    ang_train(S.N/4) = theta;
    ang_train(S.N/4 + 1) = theta;
    ang_train(S.N/4 + 2) = theta;
    %%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    while ~getappdata(G.fig,'Stop') && (time_steps <= S.N)
        % xc is the center of masses positions for the capsules
        %  __         __
        % | m0x m1x m2x | x
        % | m0y m1y m2y | y
        % | th0 th1 th2 | angle
        %  --         --
        [xc,J]      = m2c(x,E); 

        % get spring force
        cp = getappdata(G.fig,'cursorPos');
        u  = zeros(3*n,1); % I think this is the input force or impulse 
        % in the case of the flea it is a 9 by 1 vector or three 3 by 1 
        % vectors for each capsulse (as there are only 3 capsules for the 
        % flea each 3 by 1 is as follows: [vx vy angle_rate]'
        if ~isempty(cp)
            if isnan(w)
                % find nearest point on nearest capsule
                [w,rel] = nearest(xc, E.radii, cp);
            end
            xr             = [cos(xc(3,w)) -sin(xc(3,w)); sin(xc(3,w)) cos(xc(3,w))]*rel;
            xw             = xc(1:2,w) + xr;
            cursor         = [cp' xw];
            xf             = cp'-xw;
            u(3*w-2:3*w)   = D.spring*[xf; xf'*[0 -1;1 0]*xr];
%             if (mod(xc(3,2), 2*pi) < pi && mod(xc(3,3), 2*pi) < pi)
%                 u(6) = 10;
%                 u(9) = -10;
%             elseif (mod(xc(3,2), 2*pi) < pi && mod(xc(3,3), 2*pi) > pi)
%                 u(6) = 10;
%                 u(9) = -10;
%             elseif (mod(xc(3,2), 2*pi) > pi && mod(xc(3,3), 2*pi) < pi)
%                 u(6) = -10;
%                 u(9) = 10;
%             elseif (mod(xc(3,2), 2*pi) > pi && mod(xc(3,3), 2*pi) > pi)
%                 u(6) = -10;
%                 u(9) = 10;
%             end
            [mod(xc(3,2), 2*pi) mod(xc(3,3), 2*pi)]
        else
            %%%%
            u(1) = i_train(1,time_steps+1);
            u(2) = i_train(2,time_steps+1);
%             u(6) = ang_train(time_steps + 1);
%             u(9) = ang_train(time_steps + 1);
            %%%%
            cursor         = [];
            w              = nan;
        end
        
        %%%% Flea Slug Brain
        
        %%%%

        % calculate info data
        energy = 0.5*full(v'*J'*E.M*J*v);
        info  = sprintf('Energy: %-6.3g\nDynamics Calculation Time: %-4.1fms\nTotal Time Steps: %d',energy,1000*i_time,time_steps);


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
    end
    
    %%%% Plot Results
    
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
