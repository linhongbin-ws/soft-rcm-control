function [status, bools] = jnt_limit_check(q, qmin, qmax)
bool_min = ~(q>qmin);
bool_max = ~(q<qmax);
bools = -bool_min+bool_max;
status = sum(bool_min) + sum(bool_max);
end