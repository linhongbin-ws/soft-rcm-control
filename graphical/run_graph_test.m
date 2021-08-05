for index =1:1:100
    i = index/10;
    R = [ 1/2 -sqrt(3)/2 0; 
        sqrt(3)/2*cos(i) 1/2*cos(i) -sin(i);
        sqrt(3)/2*sin(i) 1/2*sin(i) cos(i)];
    t = [ 1-0.0005*i; 1+0.00005*i; 1-0.00005*i];
    T = [R t;zeros(1,3) 1];
    joints_render(T,[0,2],[0,2],[0,2]);
    pause(0.09);
end



