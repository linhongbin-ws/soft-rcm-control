classdef Visualizer < handle
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
    dvrk_version = 2
    
    %%% kinematics
    model_slave = model_Flexiv_with_stick();
    model_slave_wrist = model_instrument();
    
    sub_slave_wrist
    sub_master
    sub_slave
    
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

  end

    methods(Access = public)

        function obj = Visualizer()  % Constructor

            obj.topics()
            
        end
        
        function master_cb(obj, transform)
            rotm = quat2rotm([transform.Rotation.X transform.Rotation.Y transform.Rotation.Z transform.Rotation.W]);
            posv = [transform.Translation.X; transform.Translation.Y; transform.Translation.Z];
            if ~isempty(obj.Tt_master)
                obj.Tt_mns_1_master = obj.Tt_master;
            end
            obj.Tt_master = [rotm, posv; zeros(1,3), 1];

        end
        
        function slave_cb(obj, q)
            obj.qt_slave = q;
        end
        
        function slave_wrist_cb(obj, q)
            obj.qt_slave = q;
        end
        
        function slave_lamda_rcm_cb(obj, lamda)
            obj.lamda_rcm = lamda;
        end

        
        function topics(obj)
            if obj.dvrk_version == 2

                % master subscribe
                sub_master_cb = @(src,msg)(obj.master_cb(msg.Transform));
                obj.sub_master = rossubscriber('/MTML/measured_cp',sub_master_cb,'BufferSize',2);
        
                % slave subscribe
                sub_slave_wrist_cb = @(src,msg)(obj.slave_wrist_cb(msg.Position));
                obj.sub_slave_wrist = rossubscriber('/flexiv_wrist_get_js',sub_slave_wrist_cb,'BufferSize',2);
       
                % slave subscribe
                sub_slave_cb = @(src,msg)(obj.slave_cb(msg.Position));
                obj.sub_slave = rossubscriber('/flexiv_get_js',sub_slave_cb,'BufferSize',2);
                
                % slave subscribe
                sub_lamda_rcm_cb = @(src,msg)(obj.slave_lamda_rcm_cb(msg.Data));
                obj.sub_slave = rossubscriber('/flexiv_lamda_rcm',sub_lamda_rcm_cb,'BufferSize',2);
                    
            else
                error('not support')
            end
        end
        
        function render(obj)
            
            %%% slave kinematics
            [obj.Tt_slave_jnts, obj.Jt_slave_s] =fk_geom(obj.qt_slave, obj.model_slave.table, obj.model_slave.tip, obj.model_slave.method, true, [obj.model_slave.rcm_top_jnt_idx, obj.model_slave.rcm_tip_jnt_idx]);
            [obj.Tt_slave_wrist_jnts, obj.Jt_slave_wrist] =fk_geom(obj.qt_slave_wrist, obj.model_slave_wrist.table, obj.model_slave_wrist.tip, obj.model_slave_wrist.method, true, []);
            obj.Tt_slave = obj.Tt_slave_jnts(:,:,end);
            tmp = obj.Tt_slave_jnts;
            for k = 1:size(obj.Tt_slave_wrist_jnts,3)
                tmp = cat(3, tmp, obj.Tt_slave*obj.Tt_slave_wrist_jnts(:,:,k));
            end
            obj.Tt_slave_jnts_all = tmp;

            rcm_Tip_T = obj.Tt_slave_jnts(:,:,obj.model_slave.rcm_tip_jnt_idx+1);
            obj.rcm_p = rcm_top_T(1:3,4)*(1-obj.lamda_rcm) + rcm_Tip_T(1:3,4)*obj.lamda_rcm;
            joints_render_master_slave(obj.Tt_master, obj.Tt_slave_jnts_all, obj.xlims, obj.ylims, obj.zlims, obj.arms_offsets, obj.rcm_p, true); % visualize
        end


    end
end