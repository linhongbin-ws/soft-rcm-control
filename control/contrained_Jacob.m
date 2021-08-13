function H = contrained_Jacob(J_rcm_top, J_rcm_tip, lamda_rcm, nPlane_i, nPlane_m)
%     i = T_rcm_top(1:3,1);
%     m = T_rcm_top(1:3,2);
    left_mat = [nPlane_i.';nPlane_m.'];
    H = left_mat * (lamda_rcm*J_rcm_tip(1:3,:) + (1-lamda_rcm)*J_rcm_top(1:3,:));
end