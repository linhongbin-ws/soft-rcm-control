% get Flexiv kinematics Model with DH params

function m = model_Flexiv_DH()

    % type 0 : fixed
    % type 1 : revolute 
    % type 2 : prismatic
 
    m.table = [
        % type   alpha   a           d       theta
        %=====================================
           1      0             0         0.345         0;
           1     pi/2         0          0.065        0;
           1    -pi/2         0          0.395        pi;
           1    -pi/2        0.02      -0.055       pi;
           1      pi/2       0.02        0.385       pi;
           1      pi/2       0             0.1           pi/2;
           1      pi/2       0.11      0.055        0;
           ];
   m.tip = eye(4);
   m.method = 'DH_Modified';
   m.DOF =  sum(m.table(:,1)~=0);
end