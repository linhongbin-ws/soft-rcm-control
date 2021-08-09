% A wrapped function to run RCM Control calculation for each loop
function [qdott_slave, qdott_slave_wrist, error_transl_norms, error_rot_norms] = RCM_Control_Function(...
                                                            qt_slave, ...
                                                            qt_slave_wrist, ...  
                                                            qdott_slave, ...
                                                            Tt_master, ...
                                                            Tt_mns_1_master, ...
                                                            lamda_rcm, ...
                                                            model_slave, ...
                                                            model_slave_wrist)

    %%% slave cartesian pose
    [Tt_slave_jnts, Jt_slave_s] =fk_geom(qt_slave, model_slave.table, model_slave.tip, model_slave.method, true, [model_slave.rcm_top_jnt_idx, model_slave.rcm_tip_jnt_idx]);
    [Tt_slave_wrist_jnts, Jt_slave_wrist] =fk_geom(qt_slave_wrist, model_slave_wrist.table, model_slave_wrist.tip, model_slave_wrist.method, true, []);
    Tt_slave = Tt_slave_jnts(:,:,end);
    Tt_slave_jnts_all = Tt_slave_jnts;
    for k = 1:size(Tt_slave_wrist_jnts,3)
        Tt_slave_jnts_all = cat(3, Tt_slave_jnts_all, Tt_slave*Tt_slave_wrist_jnts(:,:,k));
    end
    
    %%% update lamda_rcm
    rcm_top_T = Tt_slave_jnts(:,:,model_slave.rcm_top_jnt_idx+1);
    rcm_tip_T = Tt_slave_jnts(:,:,model_slave.rcm_tip_jnt_idx+1);
    stick_length = norm(rcm_tip_T(1:3,4) - rcm_top_T(1:3,4));
    rcm_top_jacob = Jt_slave_s(:,:,2) ;
    rcm_tip_jacob = Jt_slave_s(:,:,3) ;
    rcm_jacob = rcm_top_jacob * (1-lamda_rcm) + rcm_tip_jacob*lamda_rcm;
    zv = dot(rcm_jacob(1:3,:)*qdott_slave,  rcm_top_T(1:3,3)); % velocity along z axis of rcm stick
    lamda_rcm  =lamda_rcm - (zv * time_delta)/stick_length;
    rcm_p = rcm_top_T(1:3,4)*(1-lamda_rcm) + rcm_tip_T(1:3,4)*lamda_rcm;
    rcm_ps = cat(3, rcm_ps, rcm_p);
    vec = rcm_ps(:,:,end) - rcm_ps(:,:,1);
    ds = [ds, norm(vec)];

    
    %%% calculate mapping
    Tt_slave_dsr = map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave, transl_scale);
    [Tt_err_slave, ~] = error_T(Tt_slave,Tt_slave_dsr); % get positional error
    [vel_master, ~] = error_T(Tt_master,Tt_mns_1_master);
    vt_dsr_slave = [map_R*vel_master(1:3);map_R*vel_master(4:6)]; % get desired velocity
    tmp = map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave_tip, transl_scale);
    [tmp, ~] = error_T(Tt_slave_jnts_all(:,:,end),tmp); % get positional error
    Tt_err_slave_wrist = [Tt_slave(1:3,1:3).'*tmp(1:3,:);Tt_slave(1:3,1:3).'*tmp(4:6,:)];
    vt_dsr_slave_wrist = [Tt_slave(1:3,1:3).'*vt_dsr_slave(1:3,:);Tt_slave(1:3,1:3).'*vt_dsr_slave(4:6,:)];

    
    % calculate contrain jacobian
    H = contrained_Jacob(Tt_slave_jnts(:,:,model_slave.rcm_top_jnt_idx+1), Jt_slave_s(:,:,2), Jt_slave_s(:,:,3), lamda_rcm);
    
   % calculate control with rcm positional tracking for redundant robot
    [qdott_slave, error_transl_norms] = control_inv_jacob_rdd_pos_rcm(Tt_err_slave, vt_dsr_slave, Jt_slave_s(:,:,1), H, lambda_transl, zeros(7,1), tracking_gain_transl); % inverse jacobian control for redundant robot

    % calculate control with rotational tracking for robot
    [qdott_slave_wrist, error_rot_norms] = control_inv_jacob_rot(Tt_err_slave_wrist,vt_dsr_slave_wrist,Jt_slave_wrist,lambda_rot, tracking_gain_rot);

end

