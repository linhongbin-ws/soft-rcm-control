% load simulated data
clear all
load(fullfile( '..','data','dvrk_mtm_psm.mat'))

addpath(genpath(fullfile('..','..','soft-rcm-control')))
%%  params

%%% general
time_delta = 0.01; % 100hz
duration = 5;
result_dir = fullfile('run')
if ~exist(result_dir, 'dir')
       mkdir(result_dir)
end

%%% graphical
 xlims = [-1.5 1.5];
 ylims = [-1.5 1.5];
 zlims = [-0.5 2.5];
arms_offsets = [0,0.5,0]; % master slave arm offset in the render figure
loops_per_plots = 10; % render every n control loop
is_video = false;
if is_video
    mp4_file_dir = fullfile(result_dir, 'render');
    video = VideoWriter(mp4_file_dir);
    open(video)
else
    video = [];
end


%%% control
map_R = []; % empty if track current R
transl_scale = 0.5; %mapping scale for translation
lamda_rcm0 = 0.5; %set inital rcm point with lamda
tracking_gain_transl = 100; %control gain for tracking PD control
lambda_transl = 0.01; % lamda = D/P, D=velocity gain, P=position gain
tracking_gain_rot = 100; %control gain for tracking PD control
lambda_rot = 0.01; % lamda = D/P, D=velocity gain, P=position gain
lamda_drift_control = 0.01;
gain_drift_control = 100;


%%% kinematics
model_slave = model_Flexiv_with_stick();
model_slave_wrist = model_instrument();
q0_slave = deg2rad([0,-15,0,-75,0,90,-45].'); % slave intial q
q0_slave_wrist =  deg2rad([0,0,0].'); 
nPlane_i = [1;0;0];
nPlane_m = [0;1;0];





%% control simulation
[T0_slave,~] = fk_geom(q0_slave,model_slave.table, model_slave.tip, model_slave.method,false,[]); % get initial slave T
[T0_slave_wrist,~] = fk_geom(q0_slave_wrist,model_slave_wrist.table, model_slave_wrist.tip, model_slave_wrist.method,false,[]); % get initial slave wrist T


Ts_master = mtm_x;
% %%% additional pseudo tracking for master
% Ts_master = [];
% for i = 1:size(mtm_x,3)
%     Ts_master = cat(3, Ts_master, mtm_x(:,:,i) * rot_T(deg2rad(i/5), 'z'));
% end


% variables
% Ts_master = mtm_x; 
qs_slave = [q0_slave];
qs_slave_wrist = [q0_slave_wrist];
T0_master = Ts_master(:,:,1);
lamda_rcm = lamda_rcm0;
qdott_slave = zeros(7,1);
rcm_ps = [];
ds = [];
error_transl_norms = [];
error_rot_norms = [];
rcm_p_fix = [];




%%% use inital R as mapping R (one way to reduce rotational tracking error)
if isempty(map_R)
    T0_slave_tip = T0_slave*T0_slave_wrist;
    map_R =  inv(T0_master(1:3,1:3)) * T0_slave_tip(1:3,1:3); %
end

%%% start simulation
for i = 1:size(Ts_master, 3)
    
    %%% update current state
    qt_slave = qs_slave(:,end);
    qt_slave_wrist = qs_slave_wrist(:,end);
    Tt_master = Ts_master(:,:,i);
    if i == 1
        Tt_mns_1_master = Ts_master(:,:,1);
    else
        Tt_mns_1_master = Ts_master(:,:,i-1);
    end
    [Tt_slave_jnts, Jt_slave_s] =fk_geom(qt_slave, model_slave.table, model_slave.tip, model_slave.method, true, [model_slave.rcm_top_jnt_idx, model_slave.rcm_tip_jnt_idx]);
    [Tt_slave_wrist_jnts, Jt_slave_wrist] =fk_geom(qt_slave_wrist, model_slave_wrist.table, model_slave_wrist.tip, model_slave_wrist.method, true, []);
    Tt_slave = Tt_slave_jnts(:,:,end);
    Tt_slave_jnts_all = Tt_slave_jnts;
    for k = 1:size(Tt_slave_wrist_jnts,3)
        Tt_slave_jnts_all = cat(3, Tt_slave_jnts_all, Tt_slave*Tt_slave_wrist_jnts(:,:,k));
    end
     rcm_Top_T = Tt_slave_jnts(:,:,model_slave.rcm_top_jnt_idx+1);
     rcm_Tip_T = Tt_slave_jnts(:,:,model_slave.rcm_tip_jnt_idx+1);
     rcm_p = rcm_Top_T(1:3,4)*(1-lamda_rcm) + rcm_Tip_T(1:3,4)*lamda_rcm;
     if i == 1
         rcm_p_fix = rcm_p;
     end
     %%% calculate lamda
     lamda_rcm = norm(rcm_Top_T(1:3,4) - rcm_p_fix)/model_slave.stick_length;
     ds = [ds, norm(rcm_p -rcm_p_fix)];
    
%     %%% update lamda_rcm
%     rcm_top_T = Tt_slave_jnts(:,:,model_slave.rcm_top_jnt_idx+1);
%     rcm_tip_T = Tt_slave_jnts(:,:,model_slave.rcm_tip_jnt_idx+1);
%     stick_length = norm(rcm_tip_T(1:3,4) - rcm_top_T(1:3,4));
%     rcm_top_jacob = Jt_slave_s(:,:,2) ;
%     rcm_tip_jacob = Jt_slave_s(:,:,3) ;
%     rcm_jacob = rcm_top_jacob * (1-lamda_rcm) + rcm_tip_jacob*lamda_rcm;
%     zv = dot(rcm_jacob(1:3,:)*qdott_slave,  rcm_top_T(1:3,3));
%     lamda_rcm  =lamda_rcm - (zv * time_delta)/stick_length;
%     rcm_p = rcm_top_T(1:3,4)*(1-lamda_rcm) + rcm_tip_T(1:3,4)*lamda_rcm;
%     rcm_ps = cat(3, rcm_ps, rcm_p);
%     vec = rcm_ps(:,:,end) - rcm_ps(:,:,1);
%     ds = [ds, norm(vec)];
    
    %%% plot render
    if(mod(i,loops_per_plots) == 1) 
        tic
         joints_render_master_slave(Tt_master, Tt_slave_jnts_all, xlims, ylims, zlims, arms_offsets, rcm_p_fix, true, video); % visualize
         toc
    end
    
    %%% calculate mapping
    Tt_slave_dsr = map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave, transl_scale);
    [Tt_err_slave, ~] = error_T(Tt_slave,Tt_slave_dsr); % get positional error
    [vel_master, ~] = error_T(Tt_master,Tt_mns_1_master);
    vt_dsr_slave = [map_R*vel_master(1:3);map_R*vel_master(4:6)]; % get desired velocity
    
%     iTt_slave = inv(Tt_slave);
%     tmp =         map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave_tip, transl_scale);
    [tmp, ~] = error_T(Tt_slave_jnts_all(:,:,end), Tt_slave_dsr); % get positional error
    Tt_err_slave_wrist = [Tt_slave(1:3,1:3).'*tmp(1:3,:);Tt_slave(1:3,1:3).'*tmp(4:6,:)];
    vt_dsr_slave_wrist = [Tt_slave(1:3,1:3).'*vt_dsr_slave(1:3,:);Tt_slave(1:3,1:3).'*vt_dsr_slave(4:6,:)];
    
    

    

    
    % calculate contrain jacobian
    H = contrained_Jacob(Jt_slave_s(:,:,2), Jt_slave_s(:,:,3), lamda_rcm, nPlane_i, nPlane_m);
    
    
%     vt_slave = control_inv_jacob_redundant(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
%      vt_slave = control_inv_jacob_rdd_pos(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
    [qdott_slave, error_norm] = control_inv_jacob_rdd_pos_rcm(Tt_err_slave, vt_dsr_slave, Jt_slave_s(:,:,1), H, lambda_transl, zeros(7,1), tracking_gain_transl); % inverse jacobian control for redundant robot
    error_transl_norms = [error_transl_norms, error_norm];
    [qdott_slave_wrist, error_norm] = control_inv_jacob_rot(Tt_err_slave_wrist,vt_dsr_slave_wrist,Jt_slave_wrist,lambda_rot, tracking_gain_rot);
    error_rot_norms = [error_rot_norms, error_norm];
    
    % RCM Drift Error controlled with PI 
      X_error_drift = rcm_p_fix- rcm_p;
      X_error_drift_proj = [X_error_drift(1:2);0];
      J_drift = Jt_slave_s(:,:,2) * (1-lamda_rcm) + Jt_slave_s(:,:,3) * lamda_rcm;
     [qdott_slave_drift_control, ~] = control_inv_jacob_rdd_pos(X_error_drift_proj, zeros(3,1), J_drift, lamda_drift_control,  zeros(7,1), gain_drift_control);
    
    qs_slave= [qs_slave, qt_slave+(qdott_slave+qdott_slave_drift_control)*time_delta]; % step velocity for flexiv
    qs_slave_wrist = [qs_slave_wrist, qt_slave_wrist+qdott_slave_wrist*time_delta]; % step velocity for wrist

end

%% show simulation result
% figure()
% fig.WindowState = 'maximized';
% plot(ds)
% title('rcm drift')
% saveas(fig,'MySimulinkDiagram.png');
% 
% fig = figure()
% fig.WindowState = 'maximized';
% plot(error_transl_norms)
% title('translational tracking error')
% 
% figure()
% plot(error_rot_norms)
% title('rotational tracking error')
% saveas(fig,'MySimulinkDiagram.png');

plot_simple('rcm drift', ds, fullfile(result_dir, 'rcm drift.png'))
plot_simple('translational tracking error', error_transl_norms, fullfile(result_dir, 'track_transl_err.png'))
plot_simple('rotational tracking error', error_rot_norms, fullfile(result_dir, 'track_rot_err.png'))

fig = figure()
fig.WindowState = 'maximized';
subplot(3,1,1)
plot(reshape(Ts_master(1,4,:), size(Ts_master,3),1))
title('master translational xyz')
subplot(3,1,2)
plot(reshape(Ts_master(2,4,:), size(Ts_master,3),1))
subplot(3,1,3)
plot(reshape(Ts_master(3,4,:), size(Ts_master,3),1))
saveas(fig,fullfile(result_dir, 'trajectory.png'));


close(video)

function plot_simple(title_str, x, save_file)
    fig = figure();
    fig.WindowState = 'maximized';
    plot(x);
    title(title_str);
    saveas(fig,save_file);
end


