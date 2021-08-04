function m = Flexiv_URDF_Model()
%     m.l_RCC = 0.4318; 
%     m.l_tool = 0.4162;
%     m.l_pitch2yaw = 0.0091;
%     m.l_yaw2ctrlpnt = 0.0102;
    
    % type 0 : fixed
    % type 1 : revolute 
    % type 2 : prismatic
 
    m.URDF = [
        % type       x           y        z            rotx        roty        rotz
        %=====================================
           0             0           0        0            0             0             -pi;  % Joint0
           1             0           0        0.135     0             0                0;  % Joint1
           1            0.0       0.03     0.210     0             0                0;  % Joint2
           1            0.0     0.035     0.205     0             0                0;  % Joint3
           1          -0.02    -0.03     0.19       0             0                0;  % Joint4
           1          0.02     -0.025   0.195      0             0                0;  % Joint5
           1          0.0         0.03     0.19       0             0                0;  % Joint6
           1         -0.055    0.070    0.11       0          -pi/2             0;  % Joint7
           ];
   m.tip = eye(4);
   m.method = 'URDF';
end