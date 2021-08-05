% get Flexiv kinematics Model

function m = model_Flexiv()

    % type 0 : fixed
    % type 1 : revolute 
    % type 2 : prismatic
 
    m.table = [
        % type       x           y        z            rotx        roty        rotz               axis
        %=====================================
           0             0           0        0            0             0             -pi                3;  % Joint0
           1             0           0        0.135     0             0                0                3;  % Joint1
           1            0.0       0.03     0.210     0             0                0                2;  % Joint2
           1            0.0     0.035     0.205     0             0                0                3;  % Joint3
           1          -0.02    -0.03     0.19       0             0                0                2;  % Joint4
           1          0.02     -0.025   0.195      0             0                0                3;  % Joint5
           1          0.0         0.03     0.19       0             0                0                2;  % Joint6
           1         -0.055    0.070    0.11       0          -pi/2             0                3;  % Joint7
           ];
   m.tip = eye(4);
   m.method = 'URDF';
   m.DOF =  sum(m.table(:,1)~=0);
end