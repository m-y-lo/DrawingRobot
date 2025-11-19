function [d1,t2,t3] = IK(T0e,c)

d3 = c(1);
d4 = c(2);

% Extract desired position
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% Prismatic joint
d1 = z;

% Planar distance
r = sqrt(x^2 + y^2);

% Solve for theta3
c3 = (r^2 - d3^2 - d4^2) / (2*d3*d4);
s3 = sqrt(1 - c3^2);
t3 = atan2(s3, c3);

% Solve for theta2
t2 = atan2(y, x) - atan2(d4*s3, d3 + d4*c3);

end
