function [status, bools] = vel_limit_check(qdot, qdot_lim)
bools = ~(abs(qdot)<=abs(qdot_lim));
status = sum(bool_min);
end