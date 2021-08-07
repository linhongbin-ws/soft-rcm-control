% load simulated data
load(fullfile( '..','data','dvrk_mtm_psm.mat'))

% params
% q0_slave = deg2rad([90;90;0;0;0;0;0]);
q0_slave = deg2rad([30;30;30;30;30;30;30]);
time_delta = 0.01; % 100hz
loops_per_plots = 10;
lambda = 1/time_delta;
duration = 5;
% map_R = [1 0 0;
%      0  1 0;
%      0   0  1];
map_R = [];
 xlims = [-0.5 0.5];
 ylims = [-0.5 1];
 zlims = [-0.5 2.5];
arms_offsets = [0,0.5,0];
transl_scale = 0.1

model_slave = model_Flexiv();
Ts_master = mtm_x;

% telep tracking control
[T0_slave,~] = fk_geom(q0_slave,model_slave.table, model_slave.tip, model_slave.method,false);
qs_slave = [q0_slave];
T0_master = mtm_x(:,:,1);

if isempty(map_R)
    map_R =  inv(T0_master(1:3,1:3)) * T0_slave(1:3,1:3);
end

for i = 2:size(Ts_master, 3)
    % get current state
    qt_slave = qs_slave(:,end);
    Tt_master = Ts_master(:,:,i);
    Tt_mns_1_master = Ts_master(:,:,i-1);
    [Tt_slave_jnts, Jt_slave_s] =fk_geom(qt_slave, model_slave.table, model_slave.tip, model_slave.method, true, [model_slave.rcm_top_jnt_idx, model_slave.rcm_tip_jnt_idx]);
    Tt_slave = Tt_slave_jnts(:,:,end);
    
    if(mod(i,loops_per_plots) == 2)
%         joints_render(Tt_slave, xlims, ylims, zlims); % visualize
         joints_render_master_slave(Tt_master, Tt_slave_jnts, xlims, ylims, zlims, arms_offsets, false); % visualize
    end
    
    % map from master to slave
    Tt_slave_dsr = map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave, transl_scale);
    [Tt_err_slave, ~] = error_T(Tt_slave,Tt_slave_dsr); % get T error
    
    [vel_master, ~] = error_T(Tt_master,Tt_mns_1_master);
    vt_dsr_slave = [map_R*vel_master(1:3);map_R*vel_master(4:6)]; % get desired velocity
    
    % calculate contrain jacobian
    
%     vt_slave = control_inv_jacob_redundant(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
     vt_slave = control_inv_jacob_rdd_pos(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
     
    qs_slave(:,end) = [qt_slave+vt_slave*time_delta]; % step velocity in simulation
    

end

% % replay MTM trajectory
% for i = 2:size(mtm_x,3)
%     if(mod(i,10) == 0)
%         T = mtm_x(:,:,i);
%         xlims = [-0.5 0.5];
%         ylims = [-0.5 0.5];
%         zlims = [-0.5 0];
%         joints_render(T, xlims, ylims, zlims);
%     end
% end

