clear all;
clear
addpath EPnP;

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
timestep = 1; % Choose the timestep that that you want to use
datapoints = feature_data(timestep,:);
wireframe = feature_points;
std_noise = 0;
[x3d_h,x2d_h,A] = data_prep(wireframe,datapoints,std_noise);
[R,T,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A);

%% Use iterative method to correct for 2D/3D issues
A_new = zeros(3,4);
A_new(1:3,1:3) = A;
for i=1:16
   X_cam(i,:) = R*feature_points(:,i);
   X_cam(i,3) = X_cam(i,3)+150;
   point(i).Ximg = project_3d_2d(A_new,X_cam(i,:)');
end

for i=1:16
   x2d_hnew(i,:) = [point(i).Ximg(1) point(i).Ximg(2) 1];
end

[R_new,T_new,Xc_new,best_solution_new]=efficient_pnp(x3d_h,x2d_hnew,A);
