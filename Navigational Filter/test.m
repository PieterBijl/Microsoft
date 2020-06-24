addpath EPnP;


std_noise = 1;
feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

timestep = 1; % Choose the timestep that that you want to use
datapoints = feature_data(timestep,:);
wireframe = feature_points;



[Xc, Tc, R, euler, quat] = EPnP(wireframe,datapoints,std_noise)