classdef teleOp < handle
  properties(Access = public)

    isStarted = false;
    psm_js_publisher;
    jointStateMsg;
    dt = 0.001;
    tStart = tic;
  end

    methods(Access = public)

        %mtm_q_initial is MTM initial joint position, a 7x1 vector. psm_q_initial is MTM initial joint position, a 6x1 vector.
        function obj = teleOp(mtm_q_initial,psm_q_initial,psm_js_publisher,jointStateMsg)  % Constructor


            if (nargin > 3)
                obj.psm_js_publisher = psm_js_publisher;
                obj.jointStateMsg = jointStateMsg;
            end

            %define your global variables here
            %for example
            %global x1;
            %x1 =  [0 0 0.18 0 0 0]';
            global MTM_q PSM_q lambda PSM_Model x0 J0 R  display_interval MTM_Model t_index;
            PSM_q = [psm_q_initial];
            MTM_q = [mtm_q_initial];
            lambda = 1/obj.dt;
            PSM_Model = PSM_DH_Model();
            MTM_Model = MTM_DH_Model();
            [x0,J0] = FK_Jacob_Geometry(psm_q_initial,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
            R = [-1 0 0;
                 0  -1 0;
                 0   0  1];
             
            display_interval = 200;
            t_index = 0;

        end

        %mtm_q is MTM joint position, a 7x1 vector
        function  [psm_q,tracking_err] = run(obj, mtm_q)

            %get your global variables
            %for example
            %global x1;
            %call your MTM forward kinematics function, transformation from MTM tip frame to PSM tip frame and PSM inverse kinematics here
            global MTM_q PSM_q lambda PSM_Model x0 J0 R  display_interval MTM_Model t_index;            
            MTM_q = [MTM_q,mtm_q];
            psm_qt = PSM_q(:,end);
            mtm_qt = mtm_q;
            mtm_qt_1 = MTM_q(:,end-1);
            [psm_xt,psm_Jt] = FK_Jacob_Geometry(psm_qt,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
            [mtm_xt_1,mtm_Jt_1] = FK_Jacob_Geometry(mtm_qt_1,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
            [mtm_xt,mtm_Jt] = FK_Jacob_Geometry(mtm_qt,MTM_Model.DH, MTM_Model.tip, MTM_Model.method);
            xd_t = MTM_to_PSM_Mapping(mtm_xt);
            [psm_xe_t, delta_theta] = T_Error(psm_xt,xd_t);
            [mtm_v_t, delta_theta] = T_Error(mtm_xt,mtm_xt_1);
            psm_vd_t = [R*mtm_v_t(1:3);R*mtm_v_t(4:6)];
            psm_qdot_t = Inv_Jacob_Control(psm_xe_t, psm_vd_t, psm_Jt, lambda);
            t_index = t_index +1;
            psm_q = psm_qt+psm_qdot_t*obj.dt;            
            PSM_q = [PSM_q, psm_q];
%            if(mod(t_index,display_interval) == 0)
%                PSM_graphical(psm_xt(1:3,1:3), psm_xt(1:3,4),i)
                t_index = 0;
%            end
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
