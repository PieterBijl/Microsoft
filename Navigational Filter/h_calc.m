function h = h_calc(x,fx,fy,n,wireframe)
% x is the state vector, fx and fy are the focal lengths of the camera, n
% is the amount of feature_points
    pixel_size = 1.1*10^-5;
    for i=1:n
        q = x(7:10);
        xyz = R_quat(q)*wireframe(:,i)+x(1:3);
        h(2*i-1,1) = xyz(1)/xyz(3)*fx+256*pixel_size;
        h(2*i,1) = xyz(2)/xyz(3)*fy+256*pixel_size;
    end
end