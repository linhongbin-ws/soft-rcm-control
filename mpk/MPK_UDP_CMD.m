function MPK_UDP_CMD(udp_socket,pitch,yaw,roll,grip)

udp_content = [int32(pitch*1000),int32(yaw*1000),int32(roll*1000),int32(grip*1000)];
write(udp_socket,udp_content,"int32","192.168.10.10",8001);

end

