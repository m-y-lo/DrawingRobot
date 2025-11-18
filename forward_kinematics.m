%Forward Kinematics
%% ii. Forward Kinematics
% ----- Declare symbolic variables -----
syms t2 t3 real % Joint angles (revolute joints)
syms d1 % Link offsets (for prismatic or mixed joints)
% ----- Define DH parameters (Standard DH convention) -----
%% i = [1 2 3 4]
alpha = [0 0 0 0]; % Link twist angles (alpha_i-1)
a = [0 0 5.5 5.5]; % Link lengths (a_i-1)
d = [d1 0 0 0]; % Link offsets (d_i)
theta = [0 t2 t3 0]; % Joint angles (theta_i)
% ----- Initialize the total transformation matrix (base to end-effector) -----
T0_ee = eye(4); % Start with identity matrix
% ----- Sequentially multiply individual link transformations -----
for i = 1:length(alpha)
fprintf('T (%d) to (%d) \n',i-1,i) % Print current frame pair
T{i} = TF(a(i),alpha(i),d(i),theta(i)) % Compute A_i^(i-1) using DH parameters
pretty(simplify(T{i})) % Display symbolic result in readableform
T0_ee = T0_ee*T{i}; % Multiply to accumulate full transform
end
%%
% ----- Display the final transformation matrix -----
fprintf('T0_ee: \n')
T0_ee
fprintf('Simplified T0_ee: \n')
simplify(T0_ee,'Steps',40) % Simplify the symbolic expression (increase Steps if
% needed)

%% Functions
% ----- Standard Denavit-Hartenberg transformation matrix -----
% Inputs: a (link length), alpha (link twist), d (link offset), theta (joint angle)
% Output: 4x4 homogeneous transformation matrix
function T = TF(a,alpha,d,theta)
T = [cos(theta) -sin(theta) 0 a
sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
0 0 0 1];
end
%% 
