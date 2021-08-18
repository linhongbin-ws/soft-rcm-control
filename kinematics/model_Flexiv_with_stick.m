% get Flexiv kinematics Model

function m = model_Flexiv_with_stick()

    m.stick_length =  0.545;

    % type 0 : fixed
    % type 1 : revolute 
    % type 2 : prismatic
 
    m.table = [
        % type       x           y        z                       rotx        roty        rotz               axis       lower_limit           upper_limit        vel_limit
        %=====================================
           0         0           0        0                         0             0             -pi                3               0                            0                        0  ;  % Joint0
           1         0           0        0.135                  0             0                0                3               -2.87979              2.87979            2.61799;  % Joint1
           1         0.0       0.03     0.210                  0             0                0                2              -2.16421                2.16421            2.61799;  % Joint2
           1         0.0     0.035     0.205                  0             0                0                3               -3.05433              3.05433             2.61799;  % Joint3
           1        -0.02    -0.03     0.19                    0             0                0                2               -2.79253              1.95477             2.61799;  % Joint4
           1          0.02     -0.025   0.195                   0             0                0                3              -3.05433              3.05433             5.23599;  % Joint5
           1          0.0         0.03     0.19                     0             0                0                2             -1.48353                4.62512            5.23599;  % Joint6
           1         -0.055       0.070    0.11                    0          -pi/2             0                3             -3.05433               3.05433            5.23599;  % Joint7
           0          0               0    0.172                    pi/2        -pi/4            0                3              0                           0                       0;   % stick top
           0          0               0   m.stick_length        0             0                0                3              0                           0                       0% stick tip
           ];
   m.tip = eye(4);
   m.method = 'URDF';
   m.DOF =  sum(m.table(:,1)~=0);
   m.rcm_top_jnt_idx = 9;
   m.rcm_tip_jnt_idx = 10;
end