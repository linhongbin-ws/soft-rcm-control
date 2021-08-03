function m = MTM_DH_Model()
    m.l_arm = 0.2794;
    m.l_forearm = 0.3048+0.0597;
    m.h = 0.1506;
    
    m.method = 'Standard';
    m.DH = [
        % type   alpha   a           d       theta
        %=====================================
           1      pi/2  0             0         -pi/2;
           1      0     m.l_arm       0         -pi/2;
           1     -pi/2  m.l_forearm   0          pi/2;
           1      pi/2  0             m.h        0;
           1     -pi/2  0             0          0;
           1      pi/2  0             0         -pi/2;
           1      0     0             0          pi/2;
           ];
    m.tip = eye(4);
end