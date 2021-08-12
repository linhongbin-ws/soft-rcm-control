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
    transl_scale = 0.2; %mapping scale for translation
    lamda_rcm0 = 0.5; %set inital rcm point with lamda
    tracking_gain_transl = 200; %control gain for tracking PD control
    lambda_transl = 0.0001; % lamda = D/P, D=velocity gain, P=position gain
    tracking_gain_rot = 100; %control gain for tracking PD control
    lambda_rot = 0.01; % lamda = D/P, D=velocity gain, P=position gain


    %%% kinematics
    model_slave = model_Flexiv_with_stick();
    model_slave_wrist = model_instrument();
    q0_slave = deg2rad([0;-15;0;-75;0;90;-45]);
    q0_slave_wrist =  deg2rad([0,0,0].'); 
    
    %%% dvrk version
    dvrk_version = 1;
    
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
    error_transl_norms = [];
    error_rot_norms = [];
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
    qt_slave_wrist_dsr
    qdott_slave_wrist
    error_transl_norm
    error_rot_norm
    rcm_p
    map_R 
    T0_slave_tip
    pub_lamda_rcm
  end

    methods(Access = public)

        function obj = teleopRCM()  % Constructor

            obj.topics('idle')
        end
        
        function master_cb(obj, transform)
            rotm = quat2rotm([transform.Rotation.X transform.Rotation.Y transform.Rotation.Z transform.Rotation.W]);
            posv = [transform.Translation.X; transform.Translation.Y; transform.Translation.Z];
            if ~isempty(obj.Tt_master)
                obj.Tt_mns_1_master = obj.Tt_master;
            end
            obj.Tt_master = [rotm, posv; zeros(1,3), 1];
        end
        
        function master_cb1(obj, Pose)
            rotm = quat2rotm([Pose.Orientation.X Pose.Orientation.Y Pose.Orientation.Z Pose.Orientation.W]);
            posv = [Pose.Position.X; Pose.Position.Y; Pose.Position.Z];
            if ~isempty(obj.Tt_master)
                obj.Tt_mns_1_master = obj.Tt_master;
            end
            obj.Tt_master = [rotm, posv; zeros(1,3), 1];

        end
        
        function slave_idle_cb(obj, q)
            obj.qt_slave = q;
        end
        
        function slave_wrist_cb(obj, q)
            obj.qt_slave_wrist = q;
        end
        
        function slave_teleop_cb(obj,q)
            obj.qt_slave = q;
            is_initial_loop_teleop = obj.is_initial_loop_teleop;
            obj.step_control();
            
            if is_initial_loop_teleop
                return
            end
            
            msg = rosmessage(obj.pub_slave);
            for i = 1:7
                msg.Position(i) = obj.qt_slave_dsr(i);
            end
            obj.pub_slave.send(msg);
            
            msg = rosmessage(obj.pub_slave_wrist);
            for i = 1:3
                msg.Position(i) = obj.qt_slave_wrist_dsr(i);
            end
            obj.pub_slave_wrist.send(msg);
            
            msg = rosmessage(obj.pub_lamda_rcm);
            msg.Data = obj.lamda_rcm;
            obj.pub_lamda_rcm.send(msg);
            
%             fprintf("publish\n")
        end

        
        function topics(obj, state)
            if obj.dvrk_version == 2

                % master subscribe
                sub_master_cb = @(src,msg)(obj.master_cb(msg.Transform));
                obj.sub_master = rossubscriber('/MTML/measured_cp',sub_master_cb,'BufferSize',2);
            elseif  obj.dvrk_version == 1
                sub_master_cb1 = @(src,msg)(obj.master_cb1(msg.Pose));
                obj.sub_master = rossubscriber('/dvrk/MTML/position_cartesian_current',sub_master_cb1,'BufferSize',2);
            else
                error('not support')
            end
        
            % slave subscribe
            sub_slave_wrist_cb = @(src,msg)(obj.slave_wrist_cb(msg.Position));
            obj.sub_slave_wrist = rossubscriber('/flexiv_wrist_get_js',sub_slave_wrist_cb,'BufferSize',2);

            % slave publish
            obj.pub_slave       = rospublisher('/flexiv_set_js','sensor_msgs/JointState');
            obj.pub_slave_wrist = rospublisher('/flexiv_wrist_set_js','sensor_msgs/JointState');
            obj.pub_lamda_rcm = rospublisher('/flexiv_lamda_rcm','std_msgs/Float64');

            if strcmp(state, 'idle')
                % slave subscribe
                sub_slave_idle_cb = @(src,msg)(obj.slave_idle_cb(msg.Position));
                obj.sub_slave = rossubscriber('/flexiv_get_js',sub_slave_idle_cb,'BufferSize',2);

            elseif strcmp(state, 'teleop')
                % slave subscribe
                sub_slave_teleop_cb = @(src,msg)(obj.slave_teleop_cb(msg.Position));
                obj.sub_slave = rossubscriber('/flexiv_get_js',sub_slave_teleop_cb,'BufferSize',2);
            else
                error('not support')
            end
                

        end
        
        function close(obj)
            obj.sub_master.NewMessageFcn = @(a, b, c)[]; 
            obj.sub_slave_wrist.NewMessageFcn = @(a, b, c)[];
            obj.sub_slave.NewMessageFcn = @(a, b, c)[];
        end
        
        
        function reset(obj)
            obj.rcm_p = [];
            obj.rcm_p0 = [];
            obj.Tt_mns_1_master = [];
            obj.Tt_master = [];
            obj.T0_master = [];
            obj.T0_master = [];
            obj.lamda_rcm = obj.lamda_rcm0;
            obj.qdott_slave = [];
            obj.map_R = [];
            

            
        end
        
        function start_teleop(obj)
            obj.reset()
            obj.is_initial_loop_teleop = true;
            obj.topics('teleop');
        end
        
        function stop_teleop(obj)
             obj.topics('idle');
        end


        function step_control(obj)
            
            if obj.is_initial_loop_teleop
                obj.q0_slave = obj.qt_slave;
                obj.q0_slave_wrist = obj.qt_slave_wrist; 
                [obj.T0_slave,~]       = fk_geom(obj.q0_slave,      obj.model_slave.table,       obj.model_slave.tip,       obj.model_slave.method,false,[]); % get initial slave T
                [obj.T0_slave_wrist,~] = fk_geom(obj.q0_slave_wrist,obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method,false,[]); % get initial slave wrist T
                obj.T0_master = obj.Tt_master; 
                
                %%% use inital R as mapping R (one way to reduce rotational tracking error)
                obj.T0_slave_tip = obj.T0_slave*obj.T0_slave_wrist;
                obj.map_R =  obj.T0_master(1:3,1:3).' * obj.T0_slave_tip(1:3,1:3); 
                
                obj.is_initial_loop_teleop = false;
         
                return
            end

            
            %%% slave kinematics
            [obj.Tt_slave_jnts, obj.Jt_slave_s]           =fk_geom(obj.qt_slave, obj.model_slave.table, obj.model_slave.tip, obj.model_slave.method, true, [obj.model_slave.rcm_top_jnt_idx, obj.model_slave.rcm_tip_jnt_idx]);
            [obj.Tt_slave_wrist_jnts, obj.Jt_slave_wrist] =fk_geom(obj.qt_slave_wrist, obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method, true, []);
            obj.Tt_slave = obj.Tt_slave_jnts(:,:,end);
            tmp = obj.Tt_slave_jnts;
            for k = 1:size(obj.Tt_slave_wrist_jnts,3)
                tmp = cat(3, tmp, obj.Tt_slave*obj.Tt_slave_wrist_jnts(:,:,k));
            end
            obj.Tt_slave_jnts_all = tmp;


            %%% mapping
            Tt_slave_mapped = map_MTM2Flexiv(obj.Tt_master, obj.map_R, obj.T0_master, obj.T0_slave, obj.transl_scale);
            [Tt_err_slave, ~] = error_T(obj.Tt_slave, Tt_slave_mapped); % get positional error
            [vel_master, ~] = error_T(obj.Tt_master,obj.Tt_mns_1_master);
            vt_slave_mapped = [obj.map_R*vel_master(1:3);obj.map_R*vel_master(4:6)]; % get desired velocity
%             tmp = map_MTM2Flexiv(obj.Tt_master, obj.map_R, obj.T0_master, obj.T0_slave_tip, obj.transl_scale);
            [tmp, ~] = error_T(obj.Tt_slave_jnts_all(:,:,end), Tt_slave_mapped); % get positional error
            Tt_err_slave_wrist = [obj.Tt_slave(1:3,1:3).'*tmp(1:3,:);obj.Tt_slave(1:3,1:3).'*tmp(4:6,:)];
            vt_slave_wrist_mapped = [obj.Tt_slave(1:3,1:3).'*vt_slave_mapped(1:3,:);obj.Tt_slave(1:3,1:3).'*vt_slave_mapped(4:6,:)];


            %%% control
            H = contrained_Jacob(obj.Tt_slave_jnts(:,:,obj.model_slave.rcm_top_jnt_idx+1), obj.Jt_slave_s(:,:,2), obj.Jt_slave_s(:,:,3), obj.lamda_rcm); % contrain jacobian
            [obj.qdott_slave, obj.error_transl_norm] = control_inv_jacob_rdd_pos_rcm(Tt_err_slave, vt_slave_mapped, obj.Jt_slave_s(:,:,1), H, obj.lambda_transl, zeros(7,1), obj.tracking_gain_transl); % inverse jacobian control for redundant robot
            [obj.qdott_slave_wrist, obj.error_rot_norm] = control_inv_jacob_rot(Tt_err_slave_wrist,vt_slave_wrist_mapped,obj.Jt_slave_wrist,obj.lambda_rot, obj.tracking_gain_rot);

            
            %%% slave desired q
            obj.qt_slave_dsr       = obj.qt_slave      +obj.qdott_slave*obj.time_delta;
            obj.qt_slave_wrist_dsr = obj.qt_slave_wrist+obj.qdott_slave_wrist*obj.time_delta;
        end


    end
end
