clc; clear; close all;
set(gcf, 'Renderer', 'painters');  % Instead of OpenGL

% --------------------------- MAP AND OBSTACLES ---------------------------
% MAP
map = binaryOccupancyMap(13, 13, 2);  % width, height, resolution (cells/meter)

% setting obstacles
for i = 0.5:0.5:8
 setOccupancy(map, [5 i], 0.5);  % (x,y) positions marked as occupied
end
for i = 1:0.5:2.5
 setOccupancy(map, [i 8], 0.5)
end
for i = 0.5:0.5:13
 setOccupancy(map, [0.5 i], 0.5)
end
for i = 1:0.5:13
 setOccupancy(map, [i 13], 0.5)
end
for i = 0.5:0.5:12.5
 setOccupancy(map, [13 i], 0.5)
end
for i = 5.5:0.5:12.5
    setOccupancy(map, [i 0.5], 0.5)
end
for i = 5.5:0.5:10
    setOccupancy(map, [i 8], 0.5)
end
for i = 3.5:0.5:4.5
   for j = 8.5:0.5:12.5
    setOccupancy(map, [j i], 0.5)
   end
end
for i = 10.5:0.5:12.5
    for j = 8.5:0.5:12.5
        setOccupancy(map, [j i], 0.5)
    end
end

% -----------------------------------------------------------------------


path = [2.00 6.00;
        10.00 2.00];

x = path(1,1);
y = path(1,2);

omega = 0;
theta = 0;

t = 0;
timestep = 0.1;

wp = 1; % selected waypoint
goalRadius = 0.1;
switchRadius = 0.3;

vizRate = rateControl(1/timestep);

ax = axes;

% LYAPUNOV CONTROLLER SETUP
k_theta = 3;                                           % TUNE THIS
k_h = 10;                                              % TUNE THIS
v = 0.6;
omega_max = 2;

% REROUTER MODULE SETUP 
threshold = 7;
half_fov = pi/3;
max_r = 1.2;
min_r = 0.6;
n_angles = 12;
n_radii = 4;
inject_dist = 0.6;
max_deflect = pi/4;

% Define the current pose for the robot [x y theta].
state = [x, y, theta]';
input = [v omega]';

objective = path(end,:)';

waypoint = path(wp+1,:)';

disttoobj = norm(state(1:2) - objective);

rerouting = false;


% ---------------------------- MAIN LOOP --------------------------------

while (disttoobj > goalRadius)
     % display for debugging
    t = t + timestep;
    disp(['--------- at t = ' num2str(t) ' -------------'])
    cla(ax);
    
    % extraction
    x = state(1);
    y = state(2);
    theta = state(3);

    % path angle calculation
    dx = path(wp+1, 1) - path(wp, 1);
    dy = path(wp+1, 2) - path(wp, 2);
    phi = mod(atan2(dy, dx), 2*pi);

    % find x and y in waypoint frame of reference
    startpoint = path(wp,:)';
    pos = state(1:2);
    new_pos = pos - startpoint;
    new_x = new_pos(1);
    new_y = new_pos(2);

    % lateral error calculation
    h_e = (-1*new_x*sin(phi)) + (new_y*cos(phi));

    % angular error calculation
    theta_e = wrapToPi(theta - phi);

    % lyapunov controller output
    input = lyapunov(k_theta, k_h, theta_e, h_e, input, omega_max);

    % EoM output
    state = kinematics(timestep, state, input);

    % update distance to objective
    disttoobj = norm(state(1:2) - objective);

    % update distance to waypoint
    disttowp = norm(state(1:2) - waypoint);
    
    % update the plot
    hold off

    % plot the path

    show(map, 'Parent', gca);
    hold (ax, 'on')
    plot(ax, path(:,1),path(:,2),"-*")
    title("Path Following Simulation")
    xlim(ax, [0 13])
    ylim(ax, [0 13])


    % plot the robot
    plot(ax, state(1), state(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2)
    quiver(ax, state(1), state(2), cos(state(3)), sin(state(3)), 0.5, 'r', 'LineWidth', 1.5)

    % set the nodes. input : generatenodes(state, half_fov, max_r, min_r, n_angles, n_radii)
    [nodes, nodes_angles] = generatenodes(state, half_fov, max_r, min_r, n_angles, n_radii);
    scatter(ax, nodes(:,1),nodes(:,2),7,'filled')

    % REROUTING MODULE
    % input : (nodes, map, nodes_angles, threshold, in_rerouting)
    % output: [out_rerouting, interfering_angles]
    [rerouting, interf_angles] = reroutechk(nodes, map, nodes_angles, threshold, rerouting);
    if rerouting
        if isempty(interf_angles)
            mean_theta = 0;
        else
            mean_theta = mean(interf_angles);
            new_wp = wpinj(state,mean_theta,inject_dist,interf_angles,max_deflect);
            path = [path(1:wp, :); new_wp; path(wp+1:end, :)];
            waypoint = path(wp+1,:)';
        end
    end

% ------ DISPLAY

    %disp(['theta = ' num2str(theta) ', phi = ' num2str(phi)]);
    %disp(['h_e = ' num2str(h_e) ', theta_e = ' num2str(theta_e)]);
    omega = input(2);
    V_dot = h_e*v*sin(theta_e)+theta_e*omega;
    disp(['V_dot = ' num2str(V_dot)]);
    %disp(['x = ' num2str(x) ', y = ' num2str(y) ', theta = ' num2str(theta)]);
    disp(['Rerouting = ' mat2str(rerouting)])
    
    
% -------- WAYPOINT SWITCHING 
    if (disttowp < switchRadius)
      wp = wp + 1;
      waypoint = path(wp+1,:)';
      rerouting = false;
    end
    if checkOccupancy(map, state(1:2)')
    disp('Robot collided with an obstacle. Terminating simulation.');
    break;
    end
    pause(0.01)
end
% --------------------------- END OF MAIN LOOP ---------------------------


% ----------------------------- KINEMATICS -------------------------------

% INPUTS    : input, state
% OUTPUTS   : state_one
% note: ...._one denotes the value after one time step
%       input       = [v, omega]
%       state       = [x, y, theta]
%       state_one   = [x_one, y_one, theta_one]

function state_one = kinematics(timestep, state, input)
    stateDot = [cos(state(3)) 0; sin(state(3)) 0; 0 1]*input;
    state_one = state + timestep*stateDot;
end

% ------------------------- LYAPUNOV CONTROLLER --------------------------

% INPUTS    : k_theta, k_h, theta_e, h_e, input, omega_max
% OUTPUTS   : input_one
% NOTE      :
%               input   = [v, omega]
%               sf      : safety factor for undefined value of sin(x)/x

function input_one = lyapunov(k_theta, k_h, theta_e, h_e, input, omega_max)
    v = input(1);
    if abs(theta_e) < 1e-6
        lastterm = 1;
    else
        lastterm = sin(theta_e)/theta_e;
    end
    omega = (-1*k_theta*theta_e) + (-1*k_h*h_e*v*lastterm);
    omega = max([min([omega omega_max]) -omega_max]);
    input_one = [v, omega]';
end

% ---------------------------- GENERATE NODES ----------------------------

% INPUTS    : state, max_radius, min_radius, half_fov
% OUTPUTS   : XY_vec

function [XY_vec, angle_vec] = generatenodes(state, half_fov, max_r, min_r, n_angles, n_radii)
    angles = linspace(state(3)-half_fov,state(3)+half_fov,n_angles);
    radii = linspace(min_r,max_r,n_radii);
    [ANGLES, RADII] = meshgrid(angles, radii);
    X = RADII .* cos(ANGLES);
    Y = RADII .* sin(ANGLES);
    X_vec = state(1) + X(:);
    Y_vec = state(2) + Y(:);
    XY_vec = [X_vec Y_vec];
    angle_vec = wrapToPi(ANGLES(:) - state(3));  % now it's relative, in [-π, π]
end


% ------------------------------- REROUTING CHECK ---------------------
% INPUTS    : nodes, map, nodes_angles, threshold, rerouting status
% OUTPUT    : interference count, rerouting status

function [out_rerouting, interfering_angles] = reroutechk(nodes, map, nodes_angles, threshold, in_rerouting)
    interfering_angles = zeros(size(nodes,1), 1);  % Preallocate
    idx = 0;  % Counter for actual inserts
    for i = 1:size(nodes,1)
        node = nodes(i,:);
        if checkOccupancy(map, node)
           idx = idx + 1;
           interfering_angles(idx) = nodes_angles(i);
       end
    end

    interfering_angles = interfering_angles(1:idx);  % Trim unused zeros
    interference_count = length(interfering_angles);
    interfering_angles = unique(interfering_angles);
    if ~in_rerouting && interference_count >= threshold
        out_rerouting = true;
    else
        out_rerouting = in_rerouting;
        interfering_angles = [];  % Clear out if not rerouting yet
    end
end


% -------------------------------- WAYPOINT INJECTION -------------------
% INPUTS    : state, sum_theta, inject_dist, interfering angles
% OUTPUTS   : new_wp
function new_wp = wpinj(state,mean_theta,inject_dist,interfering_angles,max_deflection)
    deflect_angle = sign(mean_theta)*min(abs(length(interfering_angles) * mean_theta), max_deflection);
    glob_theta = wrapToPi(state(3) - deflect_angle);
    new_wp = state(1:2) + inject_dist*[cos(glob_theta);sin(glob_theta)];
    new_wp = new_wp';
end