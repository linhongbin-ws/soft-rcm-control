pub  = rospublisher('/joint_cmds','sensor_msgs/JointState');
msg = rosmessage(pub);
q0 = zeros(7,1);

% msg.Position = q0;
% pub.send(msg);

q1 = deg2rad([0,-15,0,-75,0,90,-45].');
msg.Position = q1;
pub.send(msg);
% 
% q1 = deg2rad([0,-15,0,-75,0,90,-45].');