function PSM_T = MTM_to_PSM_Mapping(MTM_T)
    psm_pos_ref = [0;0;-0.1773];
    mtm_pos_ref = [-0.0001;-0.3639;-0.1414];
    R = [-1 ,0, 0
         0, -1, 0
         0, 0, 1];
    mtm_pos_delta = MTM_T(1:3,4)-mtm_pos_ref;
    PSM_T = [ MTM_T(1:3,1:3)*R, (R*mtm_pos_delta+psm_pos_ref);zeros(1,3),1];
end
