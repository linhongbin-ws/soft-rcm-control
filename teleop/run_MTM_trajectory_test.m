r = dvrk.arm('MTML');



start = r.setpoint_cp();
%%% generate circle trajectory
radius = 0.02;
cicle_points = 100;
rounds = 2;

traj = [];
for i = 1:cicle_points*rounds
    phase = 2*pi*i/cicle_points;
    dx = radius*sin(phase);
    dy = radius*cos(phase);
    T = start;
    T(1:2,4) = start(1:2,4) + [dx; dy]; 
    traj = cat(3, traj, T);
end


fprintf('\n')
for i = 1: size(traj,3)
    r.move_cp(traj(:,:,i)).wait();
    msg = sprintf("point (%d/%d) circle: (%d/%d)", mod(i, cicle_points),cicle_points, fix(i/cicle_points)+1, rounds);
    if i~=1
        fprintf(repmat('\b',1,strlength(msg)));
    end
    fprintf(msg);
end
fprintf('\n')

 delete(r)