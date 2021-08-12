addpath(genpath(fullfile('..','..','soft-rcm-control')));

pub_slave  = rospublisher('/MTML/measured_cp','geometry_msgs/TransformStamped');

load(fullfile( '..','data','dvrk_mtm_psm.mat'));
Ts_master = mtm_x;

rate = 50
for i=1:size(Ts_master,3)
    T = Ts_master(:,:,i);
    p = T(1:3,4);
    q = tform2quat(T);
    msg = rosmessage(pub_slave);
    msg.Transform.Translation.X = p(1);
    msg.Transform.Translation.Y = p(2);
    msg.Transform.Translation.Z = p(3);
    msg.Transform.Rotation.X = q(1);
    msg.Transform.Rotation.Y = q(2);
    msg.Transform.Rotation.Z = q(3);
    msg.Transform.Rotation.W = q(4);
    pub_slave.send(msg);
    pause(1/rate);
end