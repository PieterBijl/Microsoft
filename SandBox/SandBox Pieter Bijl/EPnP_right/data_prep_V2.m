function [x3d_h,x2d_h,A] = data_prep_V2(wireframe,datapoints,std_noise)
    % The input:
    % Wireframe should be in meters
    % Datapoints are in pixels
    % Std_noise is given in pixels
    %
    % The output:
    % x3d_h returns the world coordinates of the feature points in meters
    % x2d_h is what is projected on the imagesensor of the camera and is in
    % meters
    
    f=3.9*10^-3; %50 mm of focal length

    m=1; %f=1; %<<<<<<----------------------------
    pixel_size = 1.1*10^-5;

    u0=256; v0=256;
    A=[f 0 u0*pixel_size; 0 f v0*pixel_size; 0 0 1];
    
    std_noise=std_noise*pixel_size;
    n = length(wireframe); % Should be 16
    for i=1:n
        x3d_h(i,:)=[wireframe(:,i)', 1]; % feature points
        img_data = [0.5*datapoints(1,2*i-1)+128, 0.5*datapoints(1,2*i)+128];
%        img_data(1) = img_data(1)*0.940892149;
%        img_data(2) = img_data(2)*0.991784747;
        noise=randn(1,2)*std_noise;
        img_data_noise = pixel_size*img_data+noise;
        x2d_h(i,:) = [img_data_noise, 1]; % measurements
    end
end