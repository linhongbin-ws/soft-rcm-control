function PSM_T = MTM_to_Flexiv_Mapping(MTM_T, MTM_T_initial, PSM_T_initial)
    R = [-1 ,0, 0
         0, -1, 0
         0, 0, 1];
    PSM_rot = MTM_T(1:3,1:3)*R;  % absolute mapping 
    PSM_trans = R * (MTM_T(1:3:4)- MTM_T_initial(1:3:4)) + PSM_T_initial(1:3:4) % delta mapping w.r.t. intial position
    PSM_T = [ PSM_rot, PSM_trans;zeros(1,3),1];
end
