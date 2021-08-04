clear;
addpath('../Forward_Calculation');
addpath('../Control System');
addpath('../graphical');
psm_q_initial = [0 0 0.18 0 0 0]';
mtm_q_initial = deg2rad([0 0 0 -42.299 90 0 137.701])'
tele = teleOp(mtm_q_initial,psm_q_initial);
mtm_q_next = mtm_q_initial + deg2rad([0 0 0 1 1 1 1])'
tele.run(mtm_q_next);
mtm_q_next = mtm_q_next + + deg2rad([1 1 1 1 1 1 1])'
tele.run(mtm_q_next);
