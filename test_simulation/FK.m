function [fx,fy,fz,T] = FK(c,joint)

d3 = c(1);
d4 = c(2);

d1 = joint(1);
t2 = joint(2);
t3 = joint(3);

% Correct DH table:
% alpha, a,  d,  theta
DH = [0,   0, d1,   0;
      0,   0,  0,  t2;
      0,  d3,  0,  t3;
      0,  d4,  0,   0];

To = eye(4);
fx = 0; fy = 0; fz = 0;
T{1} = To;

for j = 1:4
    
    alpha = DH(j,1);
    a     = DH(j,2);
    d     = DH(j,3);
    theta = DH(j,4);

    Ti = [cos(theta) -sin(theta) 0 a;
          sin(theta)  cos(theta) 0 0;
          0           0          1 d;
          0           0          0 1];

    To = To * Ti;

    fx = [fx To(1,4)];
    fy = [fy To(2,4)];
    fz = [fz To(3,4)];
    T{j+1} = To;

end

end
