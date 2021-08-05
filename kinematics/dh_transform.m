function T = dh_transform(theta,d,a,alpha,method)
%take in four standard DH parameters between two consecutive frames and 
%return 4x4 homogeneous intermediate transformation matrix between
%the links
if(method == 'DH_Standard')
    T = trans_T(d,'z')*rot_T(theta,'z')*trans_T(a,'x')*rot_T(alpha,'x');
elseif(method == 'DH_Modified')
    T = trans_T(a,'x')*rot_T(alpha,'x')*trans_T(d,'z')*rot_T(theta,'z');
end

end
