clear all;
clear
addpath EPnP;

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
n = length(feature_points);
timestep = 1; % Choose the timestep that that you want to use
datapoints = feature_data(timestep,:);
wireframe = feature_points;
std_noise = 0;
[x3d_h,x2d_h,A] = data_prep(wireframe,datapoints,std_noise); %This is the 'normal' data
[x3d_h_V2,x2d_h_V2,A_V2] = data_prep_V2(wireframe,datapoints,std_noise); % This is the altered version
%This is the normal version
[R,T,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A);
[R_V2,T_V2,Xc_V2,best_solution_V2] = efficient_pnp(x3d_h_V2,x2d_h_V2,A_V2);

A_new = zeros(3,4);
A_new(1:3,1:3) = A;

%% Retrieve Xc from turning the model around
for i=1:n
   X_ver(i,:) = R*feature_points(:,i)+T; 
   X_ver_V2(i,:) = R_V2*feature_points(:,i)+T_V2;
end

eul = rad2deg(rotm2eul(R))
eul_V2 = rad2deg(rotm2eul(R_V2))
