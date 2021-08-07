% model = model_Flexiv();
model = model_Flexiv_with_stick();
q = zeros(model.DOF,1);
% jnt_idx = 1;
for jnt_idx = 1:model.DOF
    index = 1:4:20;
    index = [index, flip(index)];
    index = [index, -index];
    for i =index
        q(jnt_idx) = q(jnt_idx) + deg2rad(i/5);
        [T,Jacob] = fk_geom(q, model.table, model.tip ,model.method, true);
        joints_render(T,[-0.5,0.5],[-0.5,0.5],[0,2]);
%         pause(0.01);
    end
end
