classdef WristController < handle
  
    properties
        q_cur
        q_dsr 
        gripper_q = 0
        u
        q0 = [0;0;0];
    end
    
    methods
        function obj = WristController()
            obj.u = udp("192.168.10.10",8001);
            fopen(obj.u);
            obj.move(obj.q0);
        end
        
        function  move(obj, q)
            obj.q_dsr = q;
            x = [int32(q(1)*1000),int32(q(2)*1000),int32(q(3)*1000),int32(obj.gripper_q*1000)];
            fwrite(obj.u,x,"int32");
            obj.q_cur = obj.q_dsr; % assume current equal desire
        end
        
        function q_cur = get_current(obj)
            q_cur = obj.q_cur;
        end
        function close(obj)
            fclose(obj.u);
            clear obj.u
            delete(obj)
        end
    end
end

