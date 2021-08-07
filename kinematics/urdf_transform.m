function T = urdf_transform(x,y,z, rotx, roty, rotz)
    T = trans_T(x,'x') *trans_T(y,'y')  *trans_T(z,'z')*rot_T(rotx,'x')*rot_T(roty,'y')*rot_T(rotz,'z');
end
