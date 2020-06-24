function [x3d_h,x2d_h,A] = EPnP_data(wireframe,datapoints,std_noise)
    % The input:
    % Wireframe should be in meters
    % Datapoints are in pixels
    % Std_noise is given in pixels
    %
    % The output:
    % x3d_h returns the world coordinates of the feature points in meters
    % x2d_h is what is projected on the imagesensor of the camera and is in
    % meters
    
    f=3.9*10^-3; % focal length in meters
    pixel_size = 1.1*10^-5; % Pixel size in meters

    u0=256; v0=256; % Principal point of the camera
    A=[f 0 u0*pixel_size; 0 f v0*pixel_size; 0 0 1]; %Matrix with camera properties in meters
    
    %std_noise=std_noise*pixel_size; %Standard noise converted to meters
    n = length(wireframe); % Should be 16
    for i=1:n
        x3d_h(i,:)=[wireframe(:,i)', 1]; % feature points
        noise=randn(1,2)*std_noise;
        img_data = [0.5*datapoints(1,2*i-1)+128, 0.5*datapoints(1,2*i)+128]+noise; % The 0.5* and +128 are necessary to correct for perspective distortion
        img_data_noise = pixel_size*img_data;
        x2d_h(i,:) = [img_data_noise, 1]; % measurements
    end
end