norms = [];
for i = 1:size(X_error_drift_records,2)
    norms = [norms, norm(X_error_drift_records(:,i))];
end
idx = 4200
t = (1:idx)/100;
plot(t,norms(:,1:idx))
xlabel('time(t)')
ylabel('drift(m)')
title('RCM Drift with Portional closed-loop feedback')
set(gca,'FontSize',20)