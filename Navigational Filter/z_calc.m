function z = z_calc(datapoints,n,std_noise)
% This functions returns the measurements with perspective correction 
    pixel_size = 1.1*10^-5;
    for i=1:2*n
        z(i,1) = pixel_size*(0.5*datapoints(i)+128+std_noise);
    end
end