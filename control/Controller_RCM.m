classdef Controller_RCM < handle
    
    properties
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

    end
    
    methods
        function obj = Controller_RCM(q0_slave, q0_slave_wrist)
            [T0_slave,~] = fk_geom(q0_slave,model_slave.table, model_slave.tip, model_slave.method,false,[]); % get initial slave T
            [T0_slave_wrist,~] = fk_geom(q0_slave_wrist,model_slave_wrist.table, model_slave_wrist.tip, model_slave_wrist.method,false,[]); % get initial slave wrist T
            Ts_master = mtm_x;
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
        end
        
        function init(obj)
        end
        
        function run(obj)

        end
        
    end
end

