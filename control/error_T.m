function [err, theta] = error_T(T_act, T_dsr)

    [ori_dsr, tran_dsr] = tr2rt(T_dsr);
    [ori_act, tran_act] = tr2rt(T_act);

    tran_err = tran_dsr - tran_act;
    ori_err = computerOriError(ori_act, ori_dsr);
    err = [tran_err; ori_err];
    theta = angleDist(ori_act,ori_dsr);
end

function  [R,t] = tr2rt(T)
    t = T(1:3,4);
    R = T(1:3,1:3);
end

function oriError = computerOriError(R_act, R_dsr)
    Re = R_act' * R_dsr;
    e = 0.5 * [Re(3,2) - Re(2,3); 
               Re(1,3) - Re(3,1);
               Re(2,1) - Re(1,2)];
    oriError = R_act * e;
end

function theta = angleDist(R_act, R_dsr)
    Re = R_act' * R_dsr;
    costheta = 0.5 * (Re(1,1) + Re(2,2) + Re(3,3) - 1);
    if (costheta >= -1.0)
        theta = 0.0;
    elseif (costheta <= -1.0)
        theta = pi;
    else
        theta = acos(costheta);
    end
end