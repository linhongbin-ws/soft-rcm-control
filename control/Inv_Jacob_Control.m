function q_dot = Inv_Jacob_Control(X_error,vd,J,lambda)
    q_dot = inv(J)*(X_error*lambda+vd);
end