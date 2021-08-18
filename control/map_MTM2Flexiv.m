function Flexiv_T = map_MTM2Flexiv(MTM_T, R, MTM_T_initial, Flexiv_T_initial, transl_scale)
%     R = [-1 ,0, 0
%          0, -1, 0
%          0, 0, 1];
    Flexiv_rot = R * MTM_T(1:3,1:3);  % absolute mapping 
    Flexiv_trans = R * (MTM_T(1:3, 4)- MTM_T_initial(1:3, 4))*transl_scale + Flexiv_T_initial(1:3, 4); % delta mapping w.r.t. intial position
    Flexiv_T = [ Flexiv_rot, Flexiv_trans;zeros(1,3),1];
end
