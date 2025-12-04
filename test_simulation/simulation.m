% MAE 263A Project
% Simulation

clc;
% clf;
clear all;

%% Parameter
d3 = 0.1397; % m --> 5.5 in
d4 = 0.1397; % m --> 5.5 in
d5 = -0.01; % pen height
c = [d3 d4 d5];

% Trajectory Cartesian Space
N = 101;
t = linspace(0,2*pi,N);         % time step
% x = 0.025*cos(t) + 0.15;        % dictates the target x
% y = 0.025*sin(t);               % target y
% z = linspace(0,0.2,N);                % target z
R = [1 0 0;0 1 0;0 0 1];      % end effector rotation

%% Draw triangle
%% Trajectory Parameters
N = 300;                 % points per triangle
z_draw = 0.00;          % drawing height
lift_height = 0.02;      % lift height
cx = 0.1;                % triangle center x
cy = 0.0;                % triangle center y

%% ---- 1. Large Triangle ----
side_big = 0.10;
path_big = make_triangle(cx, cy, z_draw, side_big, N);
first_big = path_big(:,1);

%% ---- 0. Move from Start Position to First Vertex (start at z=0.02) ----
start_z   = 0.02;
start_pos = [cx; cy; start_z];

move_to_big_start = [ linspace(start_pos(1), first_big(1), 60);
                      linspace(start_pos(2), first_big(2), 60);
                      linspace(start_pos(3), first_big(3), 60) ];

%% ---- 2. Lift Up after Big Triangle ----
lift = [ linspace(path_big(1,end), path_big(1,end), 50);
         linspace(path_big(2,end), path_big(2,end), 50);
         linspace(z_draw, z_draw + lift_height, 50) ];

%% ---- 3. Move Horizontally to Start of Small Triangle ----
side_small = 0.05;
h_small    = side_small * sqrt(3)/2;

P1_small_lifted = [cx, cy - h_small/2, z_draw + lift_height];

move_to_small = [ linspace(path_big(1,end), P1_small_lifted(1), 80);
                  linspace(path_big(2,end), P1_small_lifted(2), 80);
                  linspace(z_draw + lift_height, P1_small_lifted(3), 80) ];

%% ---- 4. Move Down to Drawing Height ----
move_down = [ linspace(P1_small_lifted(1), P1_small_lifted(1), 40);
              linspace(P1_small_lifted(2), P1_small_lifted(2), 40);
              linspace(z_draw + lift_height, z_draw, 40) ];

%% ---- 5. Draw Small Triangle ----
path_small = make_triangle(cx, cy, z_draw, side_small, N);

%% ---- 6. Combine Everything ----
path_total = [ move_to_big_start, ...   % move to first big vertex
               path_big, ...            % draw big triangle
               lift, ...                % lift up
               move_to_small, ...       % move to small triangle start
               move_down, ...           % move down
               path_small ];            % draw small triangle

x = path_total(1,:);
y = path_total(2,:);
z = path_total(3,:);

%% ---- Pen Down Logic ----
pen_down = [
    zeros(1, size(move_to_big_start,2)), ... % moving to big start
    ones(1,  size(path_big,2)), ...          % draw big triangle
    zeros(1, size(lift,2)), ...              % lift (no draw)
    zeros(1, size(move_to_small,2)), ...     % move horizontally (no draw)
    zeros(1, size(move_down,2)), ...         % move down (no draw)
    ones(1,  size(path_small,2))             % draw small triangle
];

%% Joint Space
for i = 1:length(x)
    p = [x(i) y(i) z(i)]';
    T0e = [R p;0 0 0 1];
    [dis1(i),theta2(i),theta3(i)] = IK(T0e,c);
end

d1 = unwrap(dis1);
t2 = unwrap(theta2);
t3 = unwrap(theta3);

joint = [d1;t2;t3];
path = [x;y;z];

movie = 0; % create movie if 1
speed = 3; % 1 to N
filename = "triangle_color_switch";
figure(1)
for i = 1:1
    animation(c,joint,path,movie,speed, filename, pen_down)
    % animation_trace(c,joint,path,movie,speed)
end

fprintf('Done!');

%% function
function path = make_triangle(cx, cy, z, side, N)

    h = side * sqrt(3)/2;

    % Vertices (upside-down)
    P1 = [cx,          cy - h/2,  z];
    P2 = [cx - side/2, cy + h/2,  z];
    P3 = [cx + side/2, cy + h/2,  z];
    P4 = P1;

    % Point distribution
    N1 = floor(N/3);
    N2 = floor(N/3);
    N3 = N - N1 - N2;

    edge1 = [linspace(P1(1),P2(1),N1);
             linspace(P1(2),P2(2),N1);
             linspace(P1(3),P2(3),N1)];

    edge2 = [linspace(P2(1),P3(1),N2);
             linspace(P2(2),P3(2),N2);
             linspace(P2(3),P3(3),N2)];

    edge3 = [linspace(P3(1),P4(1),N3);
             linspace(P3(2),P4(2),N3);
             linspace(P3(3),P4(3),N3)];

    % Combined path
    path = [edge1 edge2 edge3];

end