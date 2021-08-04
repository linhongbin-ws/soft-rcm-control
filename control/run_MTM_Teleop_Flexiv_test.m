% load simulated data
load(fullfile( '..','data','dvrk_mtm_psm.mat'))

% init
Flexiv_q0 = deg2rad([15;15;15;15;15;15;0]);
time_delta = 0.01; % 100hz
loops_per_plots = 10
lambda = 1/time_delta;
duration = 5;
R = [1 0 0;
     0  -1 0;
     0   0  -1];
 xlims = [-0.5 0.5];
 ylims = [-0.5 0.5];
 zlims = [0 2];

% telep tracking
Flexiv_Model = Flexiv_URDF_Model();
[Flexiv_T_initial,~] = FK_Jacob_Geometry(Flexiv_q0,Flexiv_Model.URDF, Flexiv_Model.tip, Flexiv_Model.method,false);
Flexiv_qs = [Flexiv_q0];
MTM_T_initial = mtm_x(:,:,1);
for i = 2:size(mtm_x,3)
    qt = Flexiv_qs(:,end);
    mtm_xt = mtm_x(:,:,i);
    mtm_xt_1 = mtm_x(:,:,i-1);
    
    [Ts,Jt] =FK_Jacob_Geometry(qt, Flexiv_Model.URDF, Flexiv_Model.tip, Flexiv_Model.method, true);
    xt = Ts(:,:,end);
    xd_t = MTM_to_Flexiv_Mapping(mtm_x(:,:,i), R, MTM_T_initial, Flexiv_T_initial);
    [xe_t, delta_theta] = T_Error(xt,xd_t);
    [mtm_v_t, delta_theta] = T_Error(mtm_xt,mtm_xt_1);
    vd_t = [R*mtm_v_t(1:3);R*mtm_v_t(4:6)];
    qdot_t = Inv_Jacob_Redundant_Control(xe_t, vd_t, Jt, lambda, zeros(7,1));
    Flexiv_qs = [Flexiv_qs, qt+qdot_t*time_delta];
    if(mod(i,loops_per_plots) == 0)
        joints_graphical(Ts, xlims, ylims, zlims);
    end
end

% MTM trajectory
for i = 2:size(mtm_x,3)
    if(mod(i,loops_per_plots) == 0)
        endpose_graphical(Ts, xlims, ylims, zlims);
    end
end

