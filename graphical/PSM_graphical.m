function PSM_graphical(q)
    persistent chain;
    global PSM_Model;
    [T,psm_Jt] = FK_Jacob_Geometry(q,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
    R = T(1:3,1:3);
    t = T(1:3,4);
    frame_length = 0.1;
    frame_width = 1;
    frame_thick = 0.1;
    lim_Max = 0.3;
    foundFig=findobj('Tag','PSM_Fast_Plot');
    patch_length = 0.07;
    if isempty(foundFig)
        fig=figure('Color','white');
        clf(fig);
        set(fig,'Name','PSM_Trajectory');
        set(fig,'DoubleBuffer','on','Tag','PSM_Fast_Plot');
        set(fig,'Renderer','painters');
        view([127.5 15]);
        hold('on');
        hax=get(fig,'CurrentAxes');
        set(hax,'DrawMode','fast');
        grid on;
        axis_lim = [-1 1 -1 1 -1 1]
        axis(lim_Max*axis_lim);

        
        %First Plot
        T = rt2tr(R,t);
        T_origin=eye(4,4);
        title(sprintf('The position: x:%.4f, y:%.4f, z:%.4f',t(1,1),t(2,1),t(3,1)));
        set(gca,'DrawMode','fast');
        plot_link = plot3(hax,[T_origin(1,4),t(1,1)],[T_origin(2,4),t(2,1)],[T_origin(3,4),t(3,1)],'k','LineWidth',2);
        plot_patch = patch(hax,patch_length*[-1,1,1,-1],patch_length*[1,1,-1,-1],[0,0,0,0],[0.859,0.859,0.859]);
        plot_base = trplot(T_origin,hax,'length',frame_length,'arrow','width', frame_width,'thick',frame_thick,'rgb');
        plot_tip = trplot(T,hax,'length',frame_length,'arrow','width', frame_width,'thick',frame_thick,'rgb');
        chain=hggroup;
        set(plot_link,'Parent',chain);
        set(plot_tip,'Parent',chain);
    else
        hax=get(foundFig,'CurrentAxes');   
    end
    
  if ~isempty(chain)
    delete(chain);
  end

    T = rt2tr(R,t);
    T_origin=eye(4,4);
    plot_link = plot3(hax,[T_origin(1,4),t(1,1)],[T_origin(2,4),t(2,1)],[T_origin(3,4),t(3,1)],'k','LineWidth',2);
    %plot_patch = patch(hax,[-0.5,0.5,0.5,-0.5],[0.5,0.5,-0.5,-0.5],[0,0,0,0],[0.859,0.859,0.859]);
    %plot_base = trplot(T_origin,hax,'length',1,'arrow','width', 1.5,'thick',2,'rgb');
    plot_tip = trplot(T,hax,'length',frame_length,'arrow','width', frame_width,'thick',frame_thick,'rgb');
    chain=hggroup;
    set(plot_link,'Parent',chain);
    %set(plot_patch,'Parent',chain);
    %set(plot_base,'Parent',chain);
    set(plot_tip,'Parent',chain);
    title(sprintf('The position: x:%.4f, y:%.4f, z:%.4f',t(1,1),t(2,1),t(3,1)));
    drawnow;
end



