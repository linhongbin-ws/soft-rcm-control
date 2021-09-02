function [q_dot, error_norm] = control_inv_jacob_rdd_pos(X_error,vd,J,lambda, q0, gain)
    x_error_3 = X_error(1:3,:);
    vd_3 = vd(1:3,:);
    J_3 = J(1:3,:);
    pJ_3 = pinv(J_3);
    error = x_error_3+vd_3*lambda;
    error_norm = norm(error);
    q_dot = pJ_3*error*gain + (eye(size(pJ_3, 1))  - pJ_3*J_3)*q0;
end