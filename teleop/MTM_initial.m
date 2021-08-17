% pub  = rospublisher('/dvrk/MTML/set_wrench_body','geometry_msgs/Wrench');
% msg = rosmessage(pub);
% msg.Force.X = 0;
% msg.Force.Y = 0;
% msg.Force.Z = 0;
% msg.Torque.X = 0;
% msg.Torque.Y = 0;
% msg.Torque.Z = 0;
% pub.send(msg)

pub  = rospublisher('/MTML/spatial/servo_cf','geometry_msgs/WrenchStamped');
msg = rosmessage(pub);
msg.Wrench.Force.X = 0;
msg.Wrench.Force.Y = 0;
msg.Wrench.Force.Z = 0;
msg.Wrench.Torque.X = 0;
msg.Wrench.Torque.Y = 0;
msg.Wrench.Torque.Z = 0;
pub.send(msg)

pub  = rospublisher('/MTML/use_gravity_compensation','std_msgs/Bool');
msg = rosmessage(pub);
msg.Data = true;
pub.send(msg)

