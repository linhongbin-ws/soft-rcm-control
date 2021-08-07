function [q_dot, error_norm] = control_inv_jacob_rdd_pos_rcm(X_error,vd, J, H,lambda, q0, gain)
    x_error_3 = X_error(1:3,:);
    vd_3 = vd(1:3,:);
    H_3 = H;
    J_3 = J(1:3,:);
    pH_3 = pinv(H_3);
    pHH_3 = pH_3 * H_3;
    J_hat_3 = J_3*(eye(size(pHH_3))-pHH_3);
    pJ_hat_3 =  pinv(J_hat_3);
    
    error = x_error_3+vd_3*lambda;
    error_norm = norm(error);
    q_dot = pJ_hat_3*error*gain + (eye(size(pHH_3))-pHH_3)*(eye(size(pJ_hat_3,1))-pJ_hat_3*J_hat_3)*q0;
end