function T = URDFtransform(x,y,z, rotx, roty, rotz)
    T = Transl(x,'x') *Transl(y,'y')  *Transl(z,'z') *Rot(rotx,'x')*Rot(roty,'y')*Rot(rotz,'z');
end
