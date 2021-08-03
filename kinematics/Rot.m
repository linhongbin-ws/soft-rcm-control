function T = rot(theta,ax)
% take value in radian and standard axis of rotation (x,y,z)
% and return 4x4 homogeneous transformation matrix of this pure rotation
T = zeros(4);
p = zeros(3,1);
if lower(ax) == 'x'
    R = [1          0               0;...
         0      cos(theta)      -sin(theta);...
         0      sin(theta)      cos(theta)];
elseif lower(ax) == 'y'
    R = [cos(theta) 0   sin(theta);...
         0          1       0     ;...
         -sin(theta) 0  cos(theta)];
elseif lower(ax) == 'z'
    R = [cos(theta)      -sin(theta)       0;...
         sin(theta)      cos(theta)        0;...
             0               0              1];
else 
    error('not a standard axis')
    
end

T = [R p;...
    zeros(1,3) 1];
end

