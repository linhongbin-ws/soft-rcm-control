function graphical2(T_psm,T_mtm)
    % plot PSM
    [R_psm,t_psm] = tr2rt(T_psm);
    T_p = rt2tr(R_psm,t_psm);
    T_origin=eye(4,4);
    x_e = t_psm(1,1); y_e = t_psm(2,1); z_e = t_psm(3,1);
    title(sprintf('The position: x:%.4f, y:%.4f, z:%.4f',x_e,y_e,z_e));
    link = plot3([T_origin(1,4),t_psm(1,1)],[T_origin(2,4),t_psm(2,1)],[T_origin(3,4),t_psm(3,1)],'k','LineWidth',2);
    hold on;
    patch([-0.5,0.5,0.5,-0.5],[0.5,0.5,-0.5,-0.5],[0,0,0,0],[0.859,0.859,0.859]);
    hold on;
    trplot(T_origin,'length',1,'arrow','width', 1.5,'thick',2,'rgb');
    hold on;
    plot_end = trplot(T_p,'length',1,'arrow','width', 1.5,'thick',2,'rgb');
    pause(0.1);
    delete(plot_end);
    delete(link);
    grid on;
    axis([-3 3 -3 3 -3 3]);
    
    % plot MTM
    [R_mtm,t_mtm] = tr2rt(T_mtm);
    T_m = rt2tr(R_mtm,t_mtm);
    T_origin=eye(4,4);
    x_e = t_mtm(1,1); y_e = t_mtm(2,1); z_e = t_mtm(3,1);
    title(sprintf('The position: x:%.4f, y:%.4f, z:%.4f',x_e,y_e,z_e));
    link = plot3([T_origin(1,4),t_mtm(1,1)],[T_origin(2,4),t_mtm(2,1)],[T_origin(3,4),t_mtm(3,1)],'k','LineWidth',2);
    hold on;
    patch([-0.5,0.5,0.5,-0.5],[0.5,0.5,-0.5,-0.5],[0,0,0,0],[0.859,0.859,0.859]);
    hold on;
    trplot(T_origin,'length',1,'arrow','width', 1.5,'thick',2,'rgb');
    hold on;
    plot_end = trplot(T_m,'length',1,'arrow','width', 1.5,'thick',2,'rgb');
    pause(0.1);
    delete(plot_end);
    delete(link);
    grid on;
    axis([-3 3 -3 3 -3 3]);
    
    
end
function  [R,t] = tr2rt(T)
    t = T(1:3,4);
    R = T(1:3,1:3);
end


