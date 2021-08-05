classdef joints_renderer < handle
     % rendering joints of mulitple robots
    
properties
    plot_nums
    axes = {}
    chains = {}
    fig
    frame_length = 0.15;
    frame_width = 0.6;
    frame_thick = 0.03;
    lim_Max = 0.3;
end
    
methods
    function obj = joints_renderer(plot_nums)
        obj.plot_nums = plot_nums;   
        obj.init_figure();
    end
    
    function init_figure(obj)
        obj.fig=figure('Color','white');
        clf(obj.fig);
        set(obj.fig,'DoubleBuffer','on','Tag','Fast_Plot');
        set(obj.fig,'Renderer','painters');
        view([127.5 15]);
        hold('on');
        if  obj.plot_nums == 1
            obj.axes{1}=gca;
            set(obj.axes{1},'SortMethod', 'childorder')
            grid on;
            obj.chains{1} = [];
        else
            for i = 1:obj.plot_nums
                obj.axes{i} = subplot(1, obj.plot_nums, i);
                set(obj.axes{i},'SortMethod', 'childorder')
                grid on;
                obj.chains{i} = [];
            end    
        end
    end
    
    function set_limit(obj, ax_id, xlims, ylims, zlims)
        axes(obj.axes{ax_id}); % switch to specific ax
        xlim(xlims);
        ylim(ylims);
        zlim(zlims);
    end
    
    function render(obj, ax_id, Ts)        
        ax = obj.axes{ax_id}
        if ~isempty(ax.Children)
            delete(ax.Children); %delete old plot object
        end

        if ndims(Ts) == 2 % when only give endtip T
            Ts = cat(3, eye(4), Ts); % only plot base and endtip
        end
        
        axes(ax)
        chain = hggroup(ax)

        for i=1:size(Ts,3)
            T = Ts(:,:,i);
            if i~=1
                Tp = Ts(:,:,i-1);
                plot_link = plot3([Tp(1,4) T(1,4)],[Tp(2,4) T(2,4)],[Tp(3,4) T(3,4)],'k','LineWidth',2);
                set(plot_link,'Parent',chain);
            end
            plot_T = trplot(T,  'length',obj.frame_length,'labels','   ' ,'arrow','width', obj.frame_width,'thick',obj.frame_thick,'rgb','Parent',chain);
            set(plot_T,'Parent',chain);
        end
        
        obj.chains{ax_id} = chain; % keep chain object
    end
    
end
end

