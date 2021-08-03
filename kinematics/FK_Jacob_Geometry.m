function [T,Jacobian] = FK_Jacob_Geometry(q,dh_table,Tip_T,DH_method)
% return position of center of mass of ith link
    T = eye(4);
    dh_size = size(dh_table);

    Jacob_ori = [0;0;1];
    z_axis = [0;0;1];
    p_pos = [0;0;0];
    for i=1:dh_size(1)
        theta = dh_table(i,5);
        d = dh_table(i,4);
        a = dh_table(i,3);
        alpha = dh_table(i,2);
        type = dh_table(i,1);
        if type == 1
            theta = theta + q(i);
            T = T*DHtransform(theta,d,a,alpha,DH_method);
            z_axis = [z_axis,T(1:3,3)]; 
            p_pos = [p_pos,T(1:3,4)];
        elseif type ==2
            d = d + q(i);
            T= T*DHtransform(theta,d,a,alpha,DH_method);
            z_axis = [z_axis,T(1:3,3)];
            p_pos = [p_pos,T(1:3,4)];
        else
            msg = sprintf('Encounter a known Joint Type %d, it must be 1 or 2',type);
            error(msg);
        end    
    end
    
     Jacobian = [];
    if DH_method == 'Standard'
        z_axis = z_axis(:,1:end-1);
        p_pos = p_pos(:,1:end-1);
    elseif DH_method == 'Modified'
        z_axis = z_axis(:,2:end);
        p_pos = p_pos(:,2:end);
    end
 
    %Tranform from last joint frame to tip frame
    T = T*Tip_T;
    p_pos = [p_pos,T(1:3,4)];
    
    

    for i=1:dh_size(1)
       type = dh_table(i,1);
       if type == 1
          Jacobian = [Jacobian,[cross(z_axis(:,i),p_pos(:,end)-p_pos(:,i));z_axis(:,i)]];
       elseif type ==2
          Jacobian = [Jacobian,[z_axis(:,i);zeros(3,1)]];
       end
    end
end