% pub  = rospublisher('/dvrk/MTML/set_wrench_body','geometry_msgs/Wrench');
% msg = rosmessage(pub);
% msg.Force.X = 0;
% msg.Force.Y = 0;
% msg.Force.Z = 0;
% msg.Torque.X = 0;
% msg.Torque.Y = 0;
% msg.Torque.Z = 0;
% pub.send(msg)

pub  = rospublisher('/MTML/use_gravity_compensation','std_msgs/Bool');
msg = rosmessage(pub);
msg.Data = true;
pub.send(msg)

pub  = rospublisher('/MTML/spatial/servo_cf','geometry_msgs/WrenchStamped');
msg = rosmessage(pub);
msg.Wrench.Force.X = 0;
msg.Wrench.Force.Y = 0;
msg.Wrench.Force.Z = 0;
msg.Wrench.Torque.X = 0;
msg.Wrench.Torque.Y = 0;
msg.Wrench.Torque.Z = 0;
pub.send(msg)

% % 
% pub  = rospublisher('/MTML/servo_cp','geometry_msgs/TransformStamped');
% msg = rosmessage(pub);
% msg.Transform.Translation.X = Tt_master(1,4)+0.02;
% msg.Transform.Translation.Y = Tt_master(2,4);
% msg.Transform.Translation.Z = Tt_master(3,4);
% quat =  tform2quat(rot_T(deg2rad(15),'y')*Tc
% msg.Transform.Rotation.X = quat(2);
% msg.Transform.Rotation.Y = quat(3);
% msg.Transform.Rotation.Z = quat(4);
% pub.send(msg)
% 
% addpath(genpath('/home/steven/code/dvrk-2-1/src/crtk/matlab_gen/msggen'))
% addpath(genpath('/home/steven/code/dvrk-2-1/src/crtk/crtk_matlab_client'))
% addpath(genpath('/home/steven/code/dvrk-2-1/src/dvrk-ros/dvrk_matlab'))
% r = dvrk.arm('MTML');
% % to locate crtk_msgs
% addpath('/home/steven/code/dvrk-2-1/src/crtk/matlab_gen/msggen')
% addpath('/home/steven/code/dvrk-2-1/src/crtk/crtk_matlab_client')
% addpath('/home/steven/code/dvrk-2-1/src/dvrk-ros/dvrk_matlab')
% 
%  r.move_cp(rot_T(deg2rad(15),'y')*r.setpoint_cp()).wait();
% 
% 
