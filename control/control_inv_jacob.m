function q_dot = control_inv_jacob(X_error,vd,J,lambda)
    q_dot = inv(J)*(X_error*lambda+vd);
end