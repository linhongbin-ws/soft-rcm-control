function joints_render(Ts, xlims, ylims, zlims)
    persistent chain;
    frame_length = 0.15;
    frame_width = 0.6;
    frame_thick = 0.03;
    lim_Max = 0.3;
    foundFig=findobj('Tag','MTM_Fast_Plot');
    patch_length = 0.07;
    if isempty(foundFig)
        fig=figure('Color','white');
        clf(fig);
        set(fig,'Name','MTM_Trajectory');
        set(fig,'DoubleBuffer','on','Tag','MTM_Fast_Plot');
        set(fig,'Renderer','painters');
        view([127.5 15]);
        hold('on');
        hax=get(fig,'CurrentAxes');
%         set(hax,'DrawMode','fast');
        set(gca,'SortMethod', 'childorder')
        grid on;
        plot_patch = patch(hax,patch_length*[-1,1,1,-1],patch_length*[1,1,-1,-1],[0,0,0,0],[0.859,0.859,0.859]);
    else
        hax=get(foundFig,'CurrentAxes');   
    end
    
  if ~isempty(chain)
    delete(chain);
  end
  
    chain=hggroup;
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
        plot_T = trplot(T,hax,'length',frame_length,'labels','   ' ,'arrow','width', frame_width,'thick',frame_thick,'rgb');
        set(plot_T,'Parent',chain);
    end

%     title(sprintf('The position: x:%.3f, y:%.3f, z:%.3f, index %d',t(1,1),t(2,1),t(3,1),i));
    drawnow;
    margin_scale = 1.3;
    xlim(xlims)
    ylim(ylims)
    zlim(zlims)
end



