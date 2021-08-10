classdef teleopRCM < handle
  properties(Access = public)

    isStarted = false;
    psm_js_publisher;
    jointStateMsg;
    dt = 0.01;
    tStart = tic;

    %%% general
    time_delta = 0.01; % 100hz
    duration = 5;

    %%% graphical
     xlims = [-1.5 1.5];
     ylims = [-1.5 1.5];
     zlims = [-0.5 2.5];
    arms_offsets = [0,0.5,0]; % master slave arm offset in the render figure
    loops_per_plots = 10; % render every n control loop

    %%% control
    map_R = []; % empty if track current R
    transl_scale = 0.5; %mapping scale for translation
    lamda_rcm0 = 0.5; %set inital rcm point with lamda
    tracking_gain_transl = 200; %control gain for tracking PD control
    lambda_transl = 0.0001; % lamda = D/P, D=velocity gain, P=position gain
    tracking_gain_rot = 100; %control gain for tracking PD control
    lambda_rot = 0.01; % lamda = D/P, D=velocity gain, P=position gain


    %%% kinematics
    model_slave = model_Flexiv_with_stick();
    model_slave_wrist = model_instrument();
    q0_slave = deg2rad([30;30;30;30;30;30;30]); % slave intial q
    q0_slave_wrist =  deg2rad([0,0,0].'); 
    
    % variables
    qs_slave
    qs_slave_wrist
    T0_master 
    lamda_rcm
    qdott_slave
    rcm_ps = [];
    ds = [];
    error_transl_norms = [];
    error_rot_norms = [];
    map_R 
    is_init = false;
    Tt_master
    Tt_mns_1_master
    rcm_p0
    rcm_p_drift

  end

    methods(Access = public)

        function obj = teleopRCM()  % Constructor
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

            %%% use inital R as mapping R (one way to reduce rotational tracking error)
            if isempty(map_R)
                T0_slave_tip = T0_slave*T0_slave_wrist;
                map_R =  inv(T0_master(1:3,1:3)) * T0_slave_tip(1:3,1:3); %
            end

        end

        %mtm_q is MTM joint position, a 7x1 vector
        function  [psm_q,tracking_err] = run(obj, Tt_master)
            if obj.is_init
                % do stuff
                obj.Tt_mns_1_master = Tt_master;
                obj.rcm_p = [];
                obj.rcm_p0 = [];
                obj.T0_master = Tt_master;
            end
            
            obj.Tt_master = Tt_master;
            
             %%% update current state
%             obj.qt_slave = qs_slave(:,end);
%             obj.qt_slave_wrist = qs_slave_wrist(:,end);
%            Ts_master = cat(3, Ts_master, Tt_master);
            

            [Tt_slave_jnts, Jt_slave_s] =fk_geom(qt_slave, obj.model_slave.table, obj.model_slave.tip, obj.model_slave.method, true, [obj.model_slave.rcm_top_jnt_idx, obj.model_slave.rcm_tip_jnt_idx]);
            [Tt_slave_wrist_jnts, Jt_slave_wrist] =fk_geom(qt_slave_wrist, obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method, true, []);
            Tt_slave = Tt_slave_jnts(:,:,end);
            Tt_slave_jnts_all = Tt_slave_jnts;
            for k = 1:size(Tt_slave_wrist_jnts,3)
                Tt_slave_jnts_all = cat(3, Tt_slave_jnts_all, Tt_slave*Tt_slave_wrist_jnts(:,:,k));
            end

            %%% update lamda_rcm
            rcm_top_T = Tt_slave_jnts(:,:,obj.model_slave.rcm_top_jnt_idx+1);
            rcm_tip_T = Tt_slave_jnts(:,:,obj.model_slave.rcm_tip_jnt_idx+1);
            stick_length = norm(rcm_tip_T(1:3,4) - rcm_top_T(1:3,4));
            rcm_top_jacob = Jt_slave_s(:,:,2) ;
            rcm_tip_jacob = Jt_slave_s(:,:,3) ;
            rcm_jacob = rcm_top_jacob * (1-obj.lamda_rcm) + rcm_tip_jacob*obj.lamda_rcm;
            zv = dot(rcm_jacob(1:3,:)*obj.qdott_slave,  rcm_top_T(1:3,3));
            obj.lamda_rcm  =obj.lamda_rcm - (zv * obj.time_delta)/stick_length;
            obj.rcm_p = rcm_top_T(1:3,4)*(1-obj.lamda_rcm) + rcm_tip_T(1:3,4)*obj.lamda_rcm;
            if obj.is_init
                obj.rcm_p_drift = norm(obj.rcm_p-obj.rcm_p0);
            end

%             %%% plot render
%             if(mod(i,loops_per_plots) == 1)
%                  rcm_Tip_T = Tt_slave_jnts(:,:,model_slave.rcm_tip_jnt_idx+1);
%                  rcm_p = rcm_top_T(1:3,4)*(1-lamda_rcm) + rcm_Tip_T(1:3,4)*lamda_rcm;
%                  joints_render_master_slave(Tt_master, Tt_slave_jnts_all, xlims, ylims, zlims, arms_offsets, rcm_p, true); % visualize
%             end

            %%% calculate mapping
            Tt_slave_dsr = map_MTM2Flexiv(Tt_master, obj.map_R, obj.T0_master, obj.T0_slave, obj.transl_scale);
            [Tt_err_slave, ~] = error_T(obj.Tt_slave, obj.Tt_slave_dsr); % get positional error
            [vel_master, ~] = error_T(obj.Tt_master,obj.obj.Tt_mns_1_master);
            vt_dsr_slave = [obj.map_R*vel_master(1:3);obj.map_R*vel_master(4:6)]; % get desired velocity

        %     iTt_slave = inv(Tt_slave);
            tmp = map_MTM2Flexiv(Tt_master, map_R, T0_master, T0_slave_tip, transl_scale);
            [tmp, ~] = error_T(Tt_slave_jnts_all(:,:,end),tmp); % get positional error
            Tt_err_slave_wrist = [obj.Tt_slave(1:3,1:3).'*tmp(1:3,:);Tt_slave(1:3,1:3).'*tmp(4:6,:)];
            vt_dsr_slave_wrist = [obj.Tt_slave(1:3,1:3).'*vt_dsr_slave(1:3,:);Tt_slave(1:3,1:3).'*vt_dsr_slave(4:6,:)];


            % calculate contrain jacobian
            H = contrained_Jacob(Tt_slave_jnts(:,:,model_slave.rcm_top_jnt_idx+1), Jt_slave_s(:,:,2), Jt_slave_s(:,:,3), lamda_rcm);


        %     vt_slave = control_inv_jacob_redundant(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
        %      vt_slave = control_inv_jacob_rdd_pos(Tt_err_slave, vt_dsr_slave, Jt_slave, lambda, zeros(7,1)); % inverse jacobian control for redundant robot
            [qdott_slave, error_norm] = control_inv_jacob_rdd_pos_rcm(Tt_err_slave, vt_dsr_slave, Jt_slave_s(:,:,1), H, lambda_transl, zeros(7,1), tracking_gain_transl); % inverse jacobian control for redundant robot
            error_transl_norms = [error_transl_norms, error_norm];
            [qdott_slave_wrist, error_norm] = control_inv_jacob_rot(Tt_err_slave_wrist,vt_dsr_slave_wrist,Jt_slave_wrist,lambda_rot, tracking_gain_rot);
            error_rot_norms = [error_rot_norms, error_norm];
            qs_slave= [qs_slave, qt_slave+qdott_slave*time_delta]; % step velocity for flexiv
            qs_slave_wrist = [qs_slave_wrist, qt_slave_wrist+qdott_slave_wrist*time_delta]; % step velocity for wrist
            
            if obj.is_init
                obj.is_init = false;
            end
 
        end

        function  callback_update_mtm_q(obj,q)
            obj.jointStateMsg.Position = obj.run(q);
            %obj.jointStateMsg.Position
            tElapsed = toc(obj.tStart);
            if (tElapsed > 0.033)
                obj.tStart = tic;
                obj.psm_js_publisher.send(obj.jointStateMsg);
            end
        end

    end
end
