function T = urdf_transform(x,y,z, rotx, roty, rotz)
    T =rot_T(rotx,'x')*rot_T(roty,'y')*rot_T(rotz,'z')* trans_T(x,'x') *trans_T(y,'y')  *trans_T(z,'z');
end
