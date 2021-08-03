% fprintf("===========MTM plot test==========\n")
% for index =1:1:100
%     i = index/10;
%     R = [ 1/2 -sqrt(3)/2 0; 
%         sqrt(3)/2*cos(i) 1/2*cos(i) -sin(i);
%         sqrt(3)/2*sin(i) 1/2*sin(i) cos(i)];
%     t = [ 1-0.0005*i; 1+0.00005*i; 1-0.00005*i];
%     MTM_graphical(R,t, index);
%     pause(0.09);
% end


fprintf("===========PSM plot test==========\n")
global PSM_Model
PSM_Model = PSM_DH_Model()
q0 = [0 0 0 0 0 0].';
for index =1:1:100
    i = index/100;
    q0 = sin(i*1) * ones(size(q0)) + q0;
    PSM_graphical(q0);
    pause(0.09);
end

