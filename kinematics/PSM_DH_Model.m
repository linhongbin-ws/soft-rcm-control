function m = PSM_DH_Model()
    m.method = 'Modified';
    m.l_RCC = 0.4318; 
    m.l_tool = 0.4162;
    m.l_pitch2yaw = 0.0091;
    m.l_yaw2ctrlpnt = 0.0102;
    
    
    m.DH = [
        % type   alpha   a           d       theta
        %=====================================
           1      pi/2  0           0         pi/2;
           1     -pi/2  0           0        -pi/2;
           2      pi/2  0           -m.l_RCC    0;
           1      0     0           m.l_tool    0;
           1     -pi/2  0           0        -pi/2;
           1     -pi/2  m.l_pitch2yaw 0        -pi/2;
           ];
   m.tip = [0 -1  0 0;
           0   0  1 m.l_yaw2ctrlpnt;
           -1  0  0 0;
           0  0  0 1;];
end