classdef teleopRCM < handle
  properties(Access = public)

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
    transl_scale = 0.9; %mapping scale for translation
    lamda_rcm0 = 0.5; %set inital rcm point with lamda
    tracking_gain_transl = 100; %control gain for tracking PD control
    lambda_transl = 0.01; % lamda = D/P, D=velocity gain, P=position gain
    tracking_gain_rot = 100; %control gain for tracking PD control
    lambda_rot = 0.01; % lamda = D/P, D=velocity gain, P=position gain
    error_transl_norm_max = 0.1;
    error_rot_norm_max = 0.1;
    fil_ratio_slave = 0.85;
    fil_ratio_slave_wrist = 0.85;


    %%% kinematics
    model_slave = model_Flexiv_with_stick();
    model_slave_wrist = model_instrument();
    q0_slave 
    q0_slave_wrist  
    nPlane_i = [1;0;0];
    nPlane_m = [0;1;0];
    map_R = [1 0 0;
             0 1 0;
             0 0 1];
    
    %%% dvrk version
    dvrk_version = 2;
    
    
    %%% record
    is_record_master = 1;
    is_record_slave = 1;
    buffer_size = 6000;
    
    % variables
    qs_slave
    qs_slave_wrist
    qt_slave;
    qt_slave_wrist
    T0_master 
    lamda_rcm
    qdott_slave
    rcm_ps = [];
    ds = [];
    is_init = false;
    Tt_master
    Tt_mns_1_master
    rcm_p0
    rcm_p_drift
    pub_slave
    pub_slave_wrist
    sub_master
    sub_slave
    sub_slave_wrist
    is_initial_loop_teleop
    T0_slave
    T0_slave_wrist
    Tt_slave_jnts
    Tt_slave_wrist_jnts
    Jt_slave_s
    Jt_slave_wrist
    Tt_slave
    Tt_slave_jnts_all
    qt_slave_dsr
    qt_slave_dsr_fil
    qt_slave_wrist_dsr
    qt_slave_wrist_dsr_fil
    qdott_slave_wrist
    error_transl_norm
    error_rot_norm
    rcm_p 
    T0_slave_tip
    pub_lamda_rcm
    rcm_p_fix
    arm
    is_exc_jnt_lim_slave
    bools_exc_jnt_lim_slave
    is_exc_jnt_lim_slave_dsr
    bools_exc_jnt_lim_slave_dsr
    is_exc_jnt_lim_slave_wrist
    bools_exc_jnt_lim_slave_wrist
    is_exc_jnt_lim_slave_wrist_dsr
    bools_exc_jnt_lim_slave_wrist_dsr
    is_exc_transl_err
    is_exc_rot_err
    arm_name
    qt_slave_dsr_filer
    qt_slave_wrist_dsr_filer
    
    Tt_master_records = []
    qt_slave_records = []
    qt_slave_wrist_records = []
  end

    methods(Access = public)

        function obj = teleopRCM(arm_name)  
            %%% contructor %%%
            if ~strcmp(arm_name, "MTML") && ~strcmp(arm_name, "MTMR")
                error("arm type not recognize")
            end
            obj.arm_name = arm_name;
            obj.reset()
            obj.topics('idle')
            pause(0.3)
        end
        
        function record(obj)
            if obj.is_record_master
                if size(obj.Tt_master_records,3) > obj.buffer_size
                    obj.Tt_master_records = cat(3, obj.Tt_master_records(:,:,2:end), obj.Tt_master);
                else
                    obj.Tt_master_records = cat(3, obj.Tt_master_records, obj.Tt_master);
                end
            end
            if obj.is_record_slave
                if size(obj.qt_slave_records,2) > obj.buffer_size
                    obj.qt_slave_records = cat(2, obj.qt_slave_records(:,2:end), obj.qt_slave);
                else
                    obj.qt_slave_records = cat(2, obj.qt_slave_records, obj.qt_slave);
                end
                
                if size(obj.qt_slave_wrist_records,2) > obj.buffer_size
                    obj.qt_slave_wrist_records = cat(2, obj.qt_slave_wrist_records(:,2:end), obj.qt_slave_wrist);
                else
                    obj.qt_slave_wrist_records = cat(2, obj.qt_slave_wrist_records, obj.qt_slave_wrist);
                end
            end
        end
        
        function save_record(obj)
            Tt_master_records = obj.Tt_master_records;
            qt_slave_records = obj.qt_slave_records;
            qt_slave_wrist_records = obj.qt_slave_wrist_records;
            
            save('record.mat', 'Tt_master_records', 'qt_slave_records', 'qt_slave_wrist_records');
        end
        
        function master_cb(obj, transform)
            %%%  Master Arm ros topics callback (dvrk V2) %%%
            
            rotm = quat2rotm([transform.Rotation.W, transform.Rotation.X transform.Rotation.Y transform.Rotation.Z ]);
            posv = [transform.Translation.X; transform.Translation.Y; transform.Translation.Z];
            if ~isempty(obj.Tt_master)
                obj.Tt_mns_1_master = obj.Tt_master;
            end
            obj.Tt_master = [rotm, posv; zeros(1,3), 1];
        end
        
        function master_cb1(obj, Pose)
            %%%  Master Arm ros topics callback (dvrk V1) %%%
            
            rotm = quat2rotm([Pose.Orientation.W, Pose.Orientation.X Pose.Orientation.Y Pose.Orientation.Z ]);
            posv = [Pose.Position.X; Pose.Position.Y; Pose.Position.Z];
            if ~isempty(obj.Tt_master)
                obj.Tt_mns_1_master = obj.Tt_master;
            end
            obj.Tt_master = [rotm, posv; zeros(1,3), 1];

        end
        
        function slave_idle_cb(obj, q)
            %%% Slave Arm ros topic callback when not in teleoperation %%%
            
            obj.qt_slave = q;
            
            %%% send lamda_rcm
            msg = rosmessage(obj.pub_lamda_rcm);
            msg.Data = obj.lamda_rcm;
            obj.pub_lamda_rcm.send(msg);
            obj.record();
        end
        
        function slave_wrist_cb(obj, q)
            %%% Slave Wrist ros topic callback
            
            obj.qt_slave_wrist = q;
        end
        
        function is_ready = is_ready_start(obj)
            %%% check if the controller is ready to be started teleoperation %%%
           
           success_cnt_num = 50;
           thres_ratio = 0.6;
           %%% simulate the control for 100 loops
           obj.reset();
           obj.is_initial_loop_teleop = true;
           success_cnt = 0;
           is_ready = false;
           for i = 1:success_cnt_num
               pause(0.01);
                if obj.check_empty_sub_signals()
                        continue
                end
                is_initial_loop_teleop = obj.is_initial_loop_teleop;
                obj.step_control();

                %%% check status
                if isempty(obj.qt_slave_wrist_dsr) || isempty(obj.qt_slave_dsr)
                    continue
                end
                if obj.update_jnt_limit_check()
                    continue
                end
                if obj.update_tracking_err()
                    continue
                end
                
                success_cnt = success_cnt +1;

           end
           is_ready = success_cnt > success_cnt_num*thres_ratio;
        end
        
        function slave_teleop_cb(obj,q)
            %%% Slave Arm ros topic callback when in teleoperation %%%
            
            obj.qt_slave = q;
            if obj.check_empty_sub_signals()
                return
            end
            is_initial_loop_teleop = obj.is_initial_loop_teleop;
            obj.step_control();
            
            obj.record();
            
            
            %%% check status
            if isempty(obj.qt_slave_wrist_dsr) || isempty(obj.qt_slave_dsr)
                return
            end
            if obj.update_jnt_limit_check()
                obj.close()
                return
            end
            if obj.update_tracking_err()
                obj.close()
                return
            end

            
            if is_initial_loop_teleop
                return
            end
            
            %%% send slave arm commands
            msg = rosmessage(obj.pub_slave);
            for i = 1:7
                msg.Position(i) = obj.qt_slave_dsr_fil(i);
            end
            obj.pub_slave.send(msg);
            
            %%% send slave wrist commands
            msg = rosmessage(obj.pub_slave_wrist);
            for i = 1:3
                msg.Position(i) = obj.qt_slave_wrist_dsr_fil(i);
            end
            obj.pub_slave_wrist.send(msg);
            
            %%% send lamda_rcm
            msg = rosmessage(obj.pub_lamda_rcm);
            msg.Data = obj.lamda_rcm;
            obj.pub_lamda_rcm.send(msg);
            
        end

        
        function topics(obj, state)
            %%% init ros topics %%%
            
            %%% master arm ros topics
            if obj.dvrk_version == 2
                sub_master_cb = @(src,msg)(obj.master_cb(msg.Transform));
                obj.sub_master = rossubscriber(['/',obj.arm_name,'/measured_cp'],sub_master_cb,'BufferSize',2);
            elseif  obj.dvrk_version == 1
                sub_master_cb1 = @(src,msg)(obj.master_cb1(msg.Pose));
                obj.sub_master = rossubscriber(['/dvrk/',obj.arm_name,'/position_cartesian_current'],sub_master_cb1,'BufferSize',2);
            else
                error('not support')
            end
            
            % slave arm ros topics
            obj.pub_slave       = rospublisher('/joint_cmds','sensor_msgs/JointState');
            if strcmp(state, 'idle')
                % slave subscribe
                sub_slave_idle_cb = @(src,msg)(obj.slave_idle_cb(msg.Position));
                obj.sub_slave = rossubscriber('/joint_states',sub_slave_idle_cb,'BufferSize',2);

            elseif strcmp(state, 'teleop')
                % slave subscribe
                sub_slave_teleop_cb = @(src,msg)(obj.slave_teleop_cb(msg.Position));
                obj.sub_slave = rossubscriber('/joint_states',sub_slave_teleop_cb,'BufferSize',2);
            else
                error('not support')
            end
        
            %%% slave wrist ros topics
            sub_slave_wrist_cb = @(src,msg)(obj.slave_wrist_cb(msg.Position));
            obj.sub_slave_wrist = rossubscriber('/flexiv_wrist_get_js',sub_slave_wrist_cb,'BufferSize',2);
            obj.pub_slave_wrist = rospublisher('/flexiv_wrist_set_js','sensor_msgs/JointState');

            %%% lamda_rcm ros topics
            obj.pub_lamda_rcm = rospublisher('/flexiv_lamda_rcm','std_msgs/Float64');

                

        end
        
        function close(obj)
            %%% close controller %%%
            
            fprintf("close controller ...\n")
            %%% clear subscribe ros topics (one way to clear safely)
            obj.sub_master.NewMessageFcn = @(a, b, c)[]; 
            obj.sub_slave_wrist.NewMessageFcn = @(a, b, c)[];
            obj.sub_slave.NewMessageFcn = @(a, b, c)[];
        end
        
        
        function reset(obj)
            %%% reset variables %%%
            
            obj.rcm_p = [];
            obj.rcm_p0 = [];
            obj.Tt_mns_1_master = [];
            obj.Tt_master = [];
            obj.T0_master = [];
            obj.T0_master = [];
            obj.lamda_rcm = obj.lamda_rcm0;
            obj.qdott_slave = [];
%             obj.map_R = [];
            obj.Tt_master_records = [];
            obj.qt_slave_records = [];
            obj.qt_slave_wrist_records = [];
            obj.qt_slave_dsr_filer = LPF(obj.fil_ratio_slave);
            obj.qt_slave_wrist_dsr_filer = LPF(obj.fil_ratio_slave_wrist);
            obj.qt_slave_dsr_fil = [];
            obj.qt_slave_wrist_dsr_fil = [];
            
        end
        
        function start(obj)
            %%% start teleoperation control %%%
            
            obj.reset();
            obj.is_initial_loop_teleop = true;
            obj.topics('teleop');
        end
        
        function stop(obj)
            %%% stop teleoperation control %%%
            
             obj.topics('idle');
        end


        function step_control(obj)
            %%% step function for teleoperation control %%%
           
            
            %%% slave kinematics
            [obj.Tt_slave_jnts, obj.Jt_slave_s]           =fk_geom(obj.qt_slave, obj.model_slave.table, obj.model_slave.tip, obj.model_slave.method, true, [obj.model_slave.rcm_top_jnt_idx, obj.model_slave.rcm_tip_jnt_idx]);
            [obj.Tt_slave_wrist_jnts, obj.Jt_slave_wrist] =fk_geom(obj.qt_slave_wrist, obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method, true, []);
            obj.Tt_slave = obj.Tt_slave_jnts(:,:,end);
            tmp = obj.Tt_slave_jnts;
            for k = 1:size(obj.Tt_slave_wrist_jnts,3)
                tmp = cat(3, tmp, obj.Tt_slave*obj.Tt_slave_wrist_jnts(:,:,k));
            end
            obj.Tt_slave_jnts_all = tmp;
            rcm_Top_T = obj.Tt_slave_jnts(:,:,obj.model_slave.rcm_top_jnt_idx+1);
            rcm_Tip_T = obj.Tt_slave_jnts(:,:,obj.model_slave.rcm_tip_jnt_idx+1);
            obj.rcm_p = rcm_Top_T(1:3,4)*(1-obj.lamda_rcm) + rcm_Tip_T(1:3,4)*obj.lamda_rcm;

            %%% calculate lamda
            if obj.is_initial_loop_teleop
                obj.rcm_p_fix = obj.rcm_p;
            end
            obj.lamda_rcm = norm(rcm_Top_T(1:3,4) - obj.rcm_p_fix)/obj.model_slave.stick_length;
            
            %%% run at the first step control
            if obj.is_initial_loop_teleop
                obj.q0_slave = obj.qt_slave;
                obj.q0_slave_wrist = obj.qt_slave_wrist; 
                obj.T0_slave = obj.Tt_slave_jnts(:,:,end);
                obj.T0_slave_wrist = obj.Tt_slave_wrist_jnts(:,:,end);
                obj.T0_master = obj.Tt_master; 
                
%                 %%% use inital R as mapping R (one way to reduce rotational tracking error)
%                 obj.T0_slave_tip = obj.T0_slave*obj.T0_slave_wrist;
%                 obj.map_R =  obj.T0_master(1:3,1:3).' * obj.T0_slave_tip(1:3,1:3); 
                
           
                obj.is_initial_loop_teleop = false;
                return
            end


            %%% mapping
            Tt_slave_mapped = map_MTM2Flexiv(obj.Tt_master, obj.map_R, obj.T0_master, obj.T0_slave, obj.transl_scale);
            [Tt_err_slave, ~] = error_T(obj.Tt_slave, Tt_slave_mapped); % get positional error
            [vel_master, ~] = error_T(obj.Tt_master,obj.Tt_mns_1_master);
            vt_slave_mapped = [obj.map_R*vel_master(1:3);obj.map_R*vel_master(4:6)]; % get desired velocity

            [tmp, ~] = error_T(obj.Tt_slave_jnts_all(:,:,end), Tt_slave_mapped); % get positional error
            Tt_err_slave_wrist = [obj.Tt_slave(1:3,1:3).'*tmp(1:3,:);obj.Tt_slave(1:3,1:3).'*tmp(4:6,:)];
            vt_slave_wrist_mapped = [obj.Tt_slave(1:3,1:3).'*vt_slave_mapped(1:3,:);obj.Tt_slave(1:3,1:3).'*vt_slave_mapped(4:6,:)];


            %%% control
            H = contrained_Jacob(obj.Jt_slave_s(:,:,2), obj.Jt_slave_s(:,:,3), obj.lamda_rcm, obj.nPlane_i, obj.nPlane_m); % contrain jacobian
            [obj.qdott_slave, obj.error_transl_norm] = control_inv_jacob_rdd_pos_rcm(Tt_err_slave, vt_slave_mapped, obj.Jt_slave_s(:,:,1), H, obj.lambda_transl, zeros(7,1), obj.tracking_gain_transl); % inverse jacobian control for redundant robot
            [obj.qdott_slave_wrist, obj.error_rot_norm] = control_inv_jacob_rot(Tt_err_slave_wrist,vt_slave_wrist_mapped,obj.Jt_slave_wrist,obj.lambda_rot, obj.tracking_gain_rot);

            
            %%% slave desired q
            obj.qt_slave_dsr       = obj.qt_slave      +obj.qdott_slave*obj.time_delta;
            obj.qt_slave_wrist_dsr = obj.qt_slave_wrist+obj.qdott_slave_wrist*obj.time_delta;
            
            %%% filter
            obj.qt_slave_dsr_fil       = obj.qt_slave_dsr_filer.update(obj.qt_slave_dsr);
            obj.qt_slave_wrist_dsr_fil = obj.qt_slave_wrist_dsr_filer.update(obj.qt_slave_wrist_dsr);
        end
        
        function is_empty = check_empty_sub_signals(obj)
            is_empty = false;
            
            if isempty(obj.Tt_master)
                is_empty = true;
            end
            
            if isempty(obj.qt_slave)
                is_empty = true;
            end
            
                        
            if isempty(obj.qt_slave_wrist)
                is_empty = true;
            end
        end
        
        function is_abnormal = update_jnt_limit_check(obj)
            
            is_abnormal =false;
            
            %%% check slave joint limit
            qmin = obj.model_slave.table(2:8,9);
            qmax = obj.model_slave.table(2:8,10);
            [obj.is_exc_jnt_lim_slave, obj.bools_exc_jnt_lim_slave]         = jnt_limit_check(obj.qt_slave,     qmin, qmax);
%             disp("dsr")
%             disp(obj.qt_slave_dsr)
%             disp("limit")
%             disp(qmin)
            [obj.is_exc_jnt_lim_slave_dsr, obj.bools_exc_jnt_lim_slave_dsr] = jnt_limit_check(obj.qt_slave_dsr, qmin, qmax);
            if obj.is_exc_jnt_lim_slave
                fprintf("exceeds joint limits: [%d,%d,%d,%d,%d,%d,%d]\n", obj.bools_exc_jnt_lim_slave.');
                is_abnormal = true;
            end
            if obj.is_exc_jnt_lim_slave_dsr
                fprintf("exceeds joint limits: [%d,%d,%d,%d,%d,%d,%d]\n", obj.bools_exc_jnt_lim_slave_dsr.');
                is_abnormal = true;
            end
            
            %%% check slave wrist joint limit
            qmin = obj.model_slave_wrist.table(1:3,6);
            qmax = obj.model_slave_wrist.table(1:3,7);
            [obj.is_exc_jnt_lim_slave_wrist, obj.bools_exc_jnt_lim_slave_wrist]         = jnt_limit_check(obj.qt_slave_wrist,     qmin, qmax);
            [obj.is_exc_jnt_lim_slave_wrist_dsr, obj.bools_exc_jnt_lim_slave_wrist_dsr] = jnt_limit_check(obj.qt_slave_wrist_dsr, qmin, qmax);
            if obj.is_exc_jnt_lim_slave_wrist
                fprintf("exceeds joint limits: [%d,%d,%d]\n", obj.bools_exc_jnt_lim_slave_wrist.');
                is_abnormal = true;
            end
            if obj.is_exc_jnt_lim_slave_wrist_dsr
                fprintf("exceeds joint limits: [%d,%d,%d]\n", obj.bools_exc_jnt_lim_slave_wrist_dsr.');
                is_abnormal = true;
            end
        end
        
        function master_R_dsr = get_master_R_dsr(obj)
            [Tt_slave, ~]       =fk_geom(obj.qt_slave, obj.model_slave.table, obj.model_slave.tip, obj.model_slave.method, false, []);
            [Tt_slave_wrist, ~] =fk_geom(obj.qt_slave_wrist, obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method, false, []);
            Tt_slave_tip = Tt_slave*Tt_slave_wrist;
            
            master_R_dsr =  obj.map_R.' * Tt_slave_tip(1:3,1:3); 
        end
        
        function move_master_alignment(obj)

             master_R_dsr = obj.get_master_R_dsr();
             
             r = dvrk.arm(obj.arm_name);
             start = r.setpoint_cp();
             start(1:3,1:3) = master_R_dsr;
             r.move_cp(start).wait();
             delete(r)
        end
        
        function is_abnormal = update_tracking_err(obj)
            
            is_abnormal = false;
            
            %%% translational error
            obj.is_exc_transl_err = obj.error_transl_norm>obj.error_transl_norm_max;
            if obj.is_exc_transl_err
                fprintf("exceeds tracking translational error %.3f (max %.3f)\n", obj.error_transl_norm, obj.error_transl_norm_max);
                is_abnormal =true;
            end
            
            %%% rotational error
            obj.is_exc_rot_err = obj.error_rot_norm>obj.error_rot_norm_max;
            if obj.is_exc_rot_err
                fprintf("exceeds tracking rotational error %.3f (max %.3f)\n", obj.error_rot_norm, obj.error_rot_norm_max);
                is_abnormal =true;
            end
            
        end
        
        function delete(obj)
            delete(obj)
        end
       
        
    end

end
