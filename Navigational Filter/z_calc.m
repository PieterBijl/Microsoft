function z = z_calc(datapoints,n,std_noise)
% This functions returns the measurements with perspective correction 
    for i=1:2*n
        z(i,1) = datapoints(i) + randn(1,1)*std_noise;
    end
end