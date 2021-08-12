% model = model_Flexiv();
model = model_Flexiv_with_stick();
% model = model_instrument();

q = zeros(model.DOF,1);
% q = deg2rad([0;-15;0;-75;0;90;-45]);
% jnt_idx = 1;
for jnt_idx = 1:model.DOF
    index = 1:4:20;
    index = [index, flip(index)];
    index = [index, -index];
    for i =index
        q(jnt_idx) = q(jnt_idx) + deg2rad(i/5);
        [T,Jacob] = fk_geom(q, model.table, model.tip ,model.method, true,[]);
        joints_render(T,[-1,1],[-1,1],[0,2]);
%         pause(0.01);
    end
end
