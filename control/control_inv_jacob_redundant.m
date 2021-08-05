function q_dot = control_inv_jacob_redundant(X_error,vd,J,lambda, q0)
    pJ = pinv(J);
    q_dot = pJ*(X_error*lambda+vd) + (eye(size(pJ, 1))  - pJ*J)*q0;
end