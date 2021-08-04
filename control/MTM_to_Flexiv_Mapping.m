function Flexiv_T = MTM_to_Flexiv_Mapping(MTM_T, R, MTM_T_initial, Flexiv_T_initial)
%     R = [-1 ,0, 0
%          0, -1, 0
%          0, 0, 1];
    Flexiv_rot = MTM_T(1:3,1:3)*R;  % absolute mapping 
    Flexiv_trans = R * (MTM_T(1:3, 4)- MTM_T_initial(1:3, 4)) + Flexiv_T_initial(1:3, 4); % delta mapping w.r.t. intial position
    Flexiv_T = [ Flexiv_rot, Flexiv_trans;zeros(1,3),1];
end
