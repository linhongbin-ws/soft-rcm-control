function [Ts,Jacobian,Flexiv_Model] = kinematics_Flexiv(q, is_Ts)
    Flexiv_Model = Flexiv_URDF_Model();
    [Ts,Jacobian] = FK_Jacob_Geometry(q,Flexiv_Model.URDF, Flexiv_Model.tip, Flexiv_Model.method,is_Ts);
end