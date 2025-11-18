syms theta1 theta2 theta3 real
syms d1 real

% DH parameters
test_dh = [0 0 d1 0; ...
           0 0 0 theta2; ...
           5.5 0 0 theta3; ...
           5.5 0 0 0];
% Parameter ranges
d1_range = linspace(3, 5.5, 100);
theta1_range = linspace(0,2*pi, 100);
theta2_range = linspace(0,2*pi, 100);
test_map = containers.Map({'d1', 'theta2', 'theta3'},{d1_range, theta1_range,theta2_range});
modified_test_dh = [0 0 0 theta2; ...
           5.5 0 0 theta3; ...
           5.5 0 0 0];
test_map2 = containers.Map({'theta2', 'theta3'},{theta1_range,theta2_range});
% Workspace plotting function
figure(1)
plot3dworkspace(test_dh, test_map)
figure(2)
plot2dworkspace(modified_test_dh,test_map2)