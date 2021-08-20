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

udp_socket = MPK_UDP_INIT;
MPK_UDP_CMD(udp_socket,pitch,yaw,roll,grip);

MPK_UDP_CLOSE(udp_socket);
clear udp_socket