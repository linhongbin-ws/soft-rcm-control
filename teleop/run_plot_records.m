% model = model_Flexiv()
% 
% Ts = [];
% 
% for i = 1:size(qt_slave_records,2)
%     [T,Jacob] = fk_geom(qt_slave_records(:,i), model.table, model.tip ,model.method, false,[]);
%     Ts = cat(3, Ts, T);
% end
% 
% fig = figure()
% subplot(3,1,1)
% x = squeeze(Ts(1,4,:)).';
% plot(x)
% subplot(3,1,2)
% y = squeeze(Ts(1,4,:)).';
% plot(y)
% z = squeeze(Ts(1,4,:)).';
% subplot(3,1,3)
% plot(z)


ratio = 0.85
fil = LPF(ratio)
x_fil = []
for i=1:size(x,2)
    x_fil = cat(2,x_fil, fil.update(x(i)));
end

figure()
% subplot(3,1,1)
plot(x_fil)
hold on
% plot(x)
% hold off

