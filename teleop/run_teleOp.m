clear;
%addpath_all();
rosshutdown;
rosinit;
psm_q_initial = [0 0 0.18 0 0 0]';
p2 = psm('PSM2');
p2.move_joint(psm_q_initial);

[pub, jointStateMsg] = rospublisher('/dvrk/PSM1/state_joint_current','sensor_msgs/JointState');
jointStateMsg.Name = {'outer_yaw';'outer_pitch';'outer_insertion';'outer_roll';'outer_wrist_pitch';'outer_wrist_yaw'};
sub = rossubscriber('/dvrk/MTMR/state_joint_current');
pause(0.1);
mtm_q_init = sub.LatestMessage.Position;
tele = teleOp(mtm_q_init,psm_q_initial,pub,jointStateMsg);
callback = @(src,msg)(tele.callback_update_mtm_q(msg.Position));
sub = rossubscriber('/dvrk/MTMR/state_joint_current',callback,'BufferSize',100);
plot_callback = @(src,msg)(PSM_graphical(msg.Position));
sub_psm1_joint_sub = rossubscriber('/dvrk/PSM1/state_joint_current',plot_callback,'BufferSize',1);

%pause(0.1);
%rosshutdown;
%tele2 = teleOp();
%tic
%tele2.run(ones(7,1));
%toc
%tic
%tele2.run(zeros(7,1));
%toc
