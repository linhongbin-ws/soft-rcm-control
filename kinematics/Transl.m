function T = transl(d,ax)
% take value in radian and standard axis of rotation (x,y,z)
% and return 4x4 homogeneous transformation matrix of this pure rotation
T = zeros(4);
R = eye(3);
p = zeros(3,1);

if lower(ax) == 'x'
    p(1) = d;
elseif lower(ax) == 'y'
    p(2) = d;
elseif lower(ax) == 'z'
    p(3) = d;
elseif strcmp(ax,'all')
    p = reshape(d,3,1);
else 
    error('not a standard axis')
    
end

T = [R p;...
    zeros(1,3) 1];
end
