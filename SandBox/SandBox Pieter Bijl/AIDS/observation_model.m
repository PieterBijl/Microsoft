function h = observation_model(state,wireframe)
    pixel_size = 1.1*10^-5;
    fx = 2*3.9*10^-3;
    fy = fx;
    h = zeros(32,1);
    q = state(7:10);
    R = quat2rotm(q');
    n = 16;
    for i=1:n
        xyz = R*wireframe(:,i)+[state(1); state(3); state(2)];
        h(2*i-1,1) = xyz(1)/xyz(3)*fx+256*pixel_size;
        h(2*i,1) = xyz(2)/xyz(3)*fy+256*pixel_size;
    end
end