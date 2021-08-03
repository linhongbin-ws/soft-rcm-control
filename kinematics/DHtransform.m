function T = DHtransform(theta,d,a,alpha,method)
%take in four standard DH parameters between two consecutive frames and 
%return 4x4 homogeneous intermediate transformation matrix between
%the links
if(method == 'Standard')
    T = Transl(d,'z')*Rot(theta,'z')*Transl(a,'x')*Rot(alpha,'x');
elseif(method == 'Modified')
    T = Transl(a,'x')*Rot(alpha,'x')*Transl(d,'z')*Rot(theta,'z');
end

end
