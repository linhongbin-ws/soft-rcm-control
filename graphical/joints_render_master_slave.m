function joints_render_master_slave(Ts_mst, Ts_slv, xlims, ylims, zlims, arms_offsets, rcm_p, isOnlyTipFrame)
    persistent chain;
    frame_length = 0.15;
    frame_width = 0.3;
    frame_thick = 0.012;
    lim_Max = 0.3;
    foundFig=findobj('Tag','Fast_Plot');
    patch_length = 0.07;
    

    
    
    if isempty(foundFig)
        % create figure
        fig=figure('Color','white');
        clf(fig);
        set(fig,'Name','MTM_Trajectory');
        set(fig,'DoubleBuffer','on','Tag','Fast_Plot');
        set(fig,'Renderer','painters');
        view([127.5 15]);
        hold('on');
        hax=get(fig,'CurrentAxes');
        set(gca,'SortMethod', 'childorder')
        grid on;
        plot_patch_mst = patch(hax,patch_length*[-1,1,1,-1],patch_length*[1,1,-1,-1],[0,0,0,0],[0.859,0.859,0.859]);
        plot_patch_slv = patch(hax,patch_length*[-1,1,1,-1] + arms_offsets(1),patch_length*[1,1,-1,-1] + arms_offsets(2),[0,0,0,0] + arms_offsets(3),[0.859,0.859,0.859]);
    else
        hax=get(foundFig,'CurrentAxes');   
    end
    
  if ~isempty(chain)
    delete(chain);
  end
  
    chain=hggroup;
    
    Ts = Ts_mst;
    if ndims(Ts) == 2
        Ts = cat(3, eye(4), Ts);
    end
    for i=1:size(Ts,3)
        T = Ts(:,:,i);
        if i~=1
            Tp = Ts(:,:,i-1);
            plot_link = plot3(hax,[Tp(1,4) T(1,4)],[Tp(2,4) T(2,4)],[Tp(3,4) T(3,4)],'k','LineWidth',2);
            set(plot_link,'Parent',chain);
        end
        if ~isOnlyTipFrame || (isOnlyTipFrame && i==size(Ts,3))
            plot_T = trplot(T,hax,'length',frame_length,'labels','   ' ,'arrow','width', frame_width,'thick',frame_thick,'rgb');
            set(plot_T,'Parent',chain);
        end

    end
    
    Ts = Ts_slv;
    if ndims(Ts) == 2
        Ts = cat(3, eye(4), Ts);
    end
    for i=1:size(Ts,3)
        T = Ts(:,:,i);
        T(1:3,4) =  T(1:3,4) + arms_offsets.';
        if i~=1
            Tp = Ts(:,:,i-1);
            Tp(1:3,4) =  Tp(1:3,4) + arms_offsets.';
            plot_link = plot3(hax,[Tp(1,4) T(1,4)],[Tp(2,4) T(2,4)],[Tp(3,4) T(3,4)],'k','LineWidth',2);
            set(plot_link,'Parent',chain);
        end
         if ~isOnlyTipFrame || (isOnlyTipFrame && i==size(Ts,3))
            plot_T = trplot(T,hax,'length',frame_length,'labels','   ' ,'arrow','width', frame_width,'thick',frame_thick,'rgb');
            set(plot_T,'Parent',chain);
         end
    end
    
    if ~isempty(rcm_p)
        pp = rcm_p +arms_offsets.';
        plot_rcm = scatter3([pp(1)], [pp(2)], [pp(3)],...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);
        set(plot_rcm,'Parent',chain);
    end


    drawnow;
%     margin_scale = 1.3;
    xlim(xlims)
    ylim(ylims)
    zlim(zlims)
end



