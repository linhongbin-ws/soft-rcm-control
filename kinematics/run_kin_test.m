MTM_model = MTM_DH_Model;
PSM_model = PSM_DH_Model;
q_mtm = zeros(7,1);
q_psm = zeros(6,1);
% fprintf("=========== MTM result ==================\n")
% [T,Jacob] = FK_Jacob_Geometry(q_mtm, MTM_model.DH, MTM_model.tip ,MTM_model.method);
% T
% Jacob
% 
% fprintf("=========== PSM result ==================\n")
% [T,Jacob] = FK_Jacob_Geometry(q_mtm, PSM_model.DH, PSM_model.tip ,PSM_model.method);
% T
% Jacob

fprintf("=========== Flexiv result ==================\n")
Flexiv_model = Flexiv_URDF_Model();
q = zeros(7,1)
[T,Jacob] = FK_Jacob_Geometry(q, Flexiv_model.URDF, Flexiv_model.tip ,Flexiv_model.method);
T
Jacob