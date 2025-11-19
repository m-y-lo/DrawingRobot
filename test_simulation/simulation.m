% MAE 263A Project
% Simulation

clc;
% clf;
clear all;

% Main
% Parameter
d3 = 0.1397; % m --> 5.5 in
d4 = 0.1397; % m --> 5.5 in
c = [d3 d4];

% Trajectory Cartesian Space
N = 101;
t = linspace(0,2*pi,N);         % time step
x = 0.025*cos(t) + 0.15;        % dictates the target x
y = 0.025*sin(t);               % target y
z = linspace(0,0.2,N);                % target z
R = [1 0 0;0 1 0;0 0 1];      % end effector rotation

% Joint Space
for i = 1:N
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
speed = 1; % 1 to N

figure(1)
for i = 1:1
    animation(c,joint,path,movie,speed)
end