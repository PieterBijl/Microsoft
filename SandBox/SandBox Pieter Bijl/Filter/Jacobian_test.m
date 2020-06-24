clear all;

addpath EPnP;
addpath Jacobian

std_noise = 0;
feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

timestep = 1; % Choose the timestep that that you want to use
datapoints = feature_data(timestep,:);
wireframe = feature_points;

[Xc, Tc, R, euler, quat] = EPnP(wireframe,datapoints,std_noise);
w = [-0.0873;-0.1489;0.0262]; 
x = [Tc; zeros(3,1); quat; w];
fx = 3.9*10^-3;
fy = 3.9*10^-3;
n = 16;
H = Jacobian(n,x, fx, fy,wireframe);