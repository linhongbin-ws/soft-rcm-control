clear
clc

pitch_deg = 0;   %Degree: -75~75 
yaw_deg = 0;   %Degree: -90~90
roll_deg = 0;   %Degree: -270~270
grip_deg = 0;     %Degree: 0~90
pitch = deg2rad(pitch_deg);
yaw = deg2rad(yaw_deg);
roll = deg2rad(roll_deg);
grip = deg2rad(grip_deg);

u = udpport("datagram","IPV4","LocalHost","192.168.10.20");

x = [int32(pitch*1000),int32(yaw*1000),int32(roll*1000),int32(grip*1000)]
write(u,x,"int32","192.168.10.10",8001);

flush(u,"output");
clear u