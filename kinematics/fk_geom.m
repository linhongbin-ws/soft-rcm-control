function [T_a,J_a] = fk_geom(q,table,Tip_T,method,is_Ts, jacob_jnts)
% calculate tip homongenous matrix and jacobian matrix using geometry
% method
    Ts = [];
    
    
    T = eye(4);
    Ts = cat(3, Ts, T);
    dh_size = size(table);

    Jacob_ori = [0;0;1];
    z_axis = [0;0;1];
    p_pos = [0;0;0];
    
    % calcuate forward kinematics
    if strcmp(method, "DH_Standard") || strcmp(method, "DH_Modified") 
        for i=1:dh_size(1)
            theta = table(i,5);
            d = table(i,4);
            a = table(i,3);
            alpha = table(i,2);
            type = table(i,1);
            if type == 1
                theta = theta + q(i);
                T = T*dh_transform(theta,d,a,alpha,method);
                Ts = cat(3, Ts, T);
                z_axis = [z_axis,T(1:3,3)]; 
                p_pos = [p_pos,T(1:3,4)];
            elseif type ==2
                d = d + q(i);
                T= T*dh_transform(theta,d,a,alpha,method);
                Ts = cat(3, Ts, T);
                z_axis = [z_axis,T(1:3,3)];
                p_pos = [p_pos,T(1:3,4)];
            else
                msg = sprintf('Encounter a known Joint Type %d, it must be 1 or 2',type);
                error(msg);
            end    
        end
        
    elseif strcmp(method, "URDF")
        rev_idx = 0; % reverse index
        for i=1:size(table, 1)
            type = table(i, 1);
            urdfs = table(i,2:7);
            if type == 1
                urdfs(table(i,8)+3) =  urdfs(table(i,8)+3) +  q(i-rev_idx); % plus 3 to move to angle index
            elseif  type ==2
                urdfs(table(i,8)) =  urdfs(table(i,8)) +  q(i-rev_idx); % plus 3 to move to angle index
%                 x = table(i, 2);  y = table(i, 3); z = table(i, 4);
%                 rotx = tmp(); roty = table(i,6); rotz = table(i,7);
%                 rotz = rotz + q(i-rev_idx);
%             elseif type ==2
%                 z = z + q(i-rev_idx);
            elseif type ==0
                rev_idx=rev_idx+1;
            else
                error('not support type')
            end
            
            T = T*urdf_transform(urdfs(1), urdfs(2), urdfs(3), urdfs(4), urdfs(5), urdfs(6));
           Ts = cat(3, Ts, T);
            if type~=0
                z_axis = [z_axis,T(1:3,table(i,8))];  % moving axis can be x, y,z, which defined by table(i,8)
                p_pos = [p_pos,T(1:3,4)];
            end
        end        
    else
        error("not support method")
    end
    

    if strcmp(method, "DH_Standard")
        z_axis = z_axis(:,1:end-1);
        p_pos = p_pos(:,1:end-1);
    elseif  strcmp(method, "DH_Modified") || strcmp(method, "URDF")
        z_axis = z_axis(:,2:end);
        p_pos = p_pos(:,2:end);
    end
 
    %Tranform from last joint frame to tip frame
    T = T*Tip_T;
   Ts = cat(3, Ts, T);
   
   % calculate jacobians
   Jacobian = jacob(table, z_axis, p_pos,  size(table, 1), T);
   if  isempty(jacob_jnts)
       J_a = Jacobian;
   else 
       J_a = [Jacobian];
       for k = jacob_jnts
           Jacobian = jacob(table, z_axis, p_pos,  k, Ts(:,:,k+1));
           J_a = cat(3, J_a, Jacobian);
       end
   end
    
%     rev_idx = 0; % reverse index
%     for j=1:size(table, 1)
%        type = table(j,1);
%        i = j-rev_idx;
%        if type == 0
%            rev_idx = rev_idx +1;
%        elseif type == 1
%           Jacobian = [Jacobian,[cross(z_axis(:,i),p_pos(:,end)-p_pos(:,i));z_axis(:,i)]];
%        elseif type ==2
%           Jacobian = [Jacobian,[z_axis(:,i);zeros(3,1)]];
%        end
%     end
    
   if is_Ts
       T_a = Ts; 
   else
       T_a = Ts(:,:, end);
   end
end

function Jacobian = jacob(table, z_axis, p_pos,  table_rows_Jacob, T_Jacob)
    Jacobian = [];
    rev_idx = 0; % reverse index
    for j=1:size(table,1)
       type = table(j,1);
       i = j-rev_idx;
       if type == 0
           rev_idx = rev_idx +1;
       elseif type == 1
          if j>table_rows_Jacob
             Jacobian = [Jacobian,zeros(6,1)];
          else
            Jacobian = [Jacobian,[cross(z_axis(:,i),T_Jacob(1:3,4)-p_pos(:,i));z_axis(:,i)]];
          end
       elseif type ==2
         if j>table_rows_Jacob
             Jacobian = [Jacobian,zeros(6,1)];
         else
                Jacobian = [Jacobian,[z_axis(:,i);zeros(3,1)]];
         end
       end
    end
end