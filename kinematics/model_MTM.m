% get MTM kinematics Model

function m = model_MTM()
    % params
    m.l_arm = 0.2794;
    m.l_forearm = 0.3048+0.0597;
    m.h = 0.1506;
    
    m.method = 'DH_Standard';
    m.table = [
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
    m.DOF =  sum(m.table(:,1)~=0);
end