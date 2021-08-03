function
PSM_q0 = zeros(6,1);
PSM_q = [PSM_q0];
time_delta = 0.01;
lambda = 1/time_delta;
duration = 5;
PSM_Model = PSM_DH_Model();
[x0,J0] = FK_Jacob_Geometry(PSM_q0,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);

d_size = size(mtm_x);
for i = 1:d_size(3)-1
    qt = PSM_q(:,end);
    [xt,Jt] = FK_Jacob_Geometry(qt,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
    xd_t = MTM_to_PSM_Mapping(mtm_x(:,:,i));
    [xe_t, delta_theta] = T_Error(xt,xd_t);
    vd_t = psm_xdot_dsr(:,i);
    qdot_t = Inv_Jacob_Control(xe_t, vd_t, Jt, lambda);
    PSM_q = [PSM_q, qt+qdot_t*time_delta];
    if(mod(i,50) == 0)
        PSM_graphical(xt(1:3,1:3), xt(1:3,4),i);
    end
    if(i == d_size(3)-1)
        i = d_size(3)-1;
    end
end


end


