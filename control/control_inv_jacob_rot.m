function [q_dot, error_norm] = control_inv_jacob_rot(X_error,vd,J,lambda,gain)
    X_error_3 = X_error(4:6,:);
    vd_3 = vd(4:6,:);
    J_3 = J(4:6,:);
    error = X_error_3+vd_3*lambda;
    error_norm = norm(error);
    q_dot = inv(J_3)*error*gain;
end