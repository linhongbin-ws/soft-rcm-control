fprintf("=========== MTM result ==================\n")
model = model_MTM();
q0 = zeros(7,1);
[T,Jacob] = fk_geom(q0, model.table, model.tip ,model.method, false);
T
Jacob

fprintf("=========== PSM result ==================\n")
model = model_PSM();
q0 = zeros(6,1);
[T,Jacob] = fk_geom(q0, model.table, model.tip ,model.method, false);
T
Jacob

fprintf("=========== Flexiv result ==================\n")
model = model_Flexiv();
q0 = zeros(7,1);
[T,Jacob] = fk_geom(q0, model.table, model.tip ,model.method, false);
T
Jacob