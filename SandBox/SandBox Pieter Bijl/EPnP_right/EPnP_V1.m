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
[x3d_h,x2d_h,A] = data_prep(wireframe,datapoints,std_noise);
[R,T,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A);
A_new = zeros(3,4);
A_new(1:3,1:3) = A;

for i=1:n
   Xcam_test(i,1) = datapoints(2*i-1)*0.2037;
   Xcam_test(i,2) = datapoints(2*i)*0.2037;
   Xcam_test(i,3) = 150;
   Xcam_h = [Xcam_test(i,:)'; 1];
   Ximg_h=A_new*Xcam_h;

   Ximg(i,1)=Ximg_h(1)/Ximg_h(3)
   Ximg(i,2)=Ximg_h(2)/Ximg_h(3)
   x2d_h_test(i,:) = [Ximg(i,:) 1];
end

[R_test,T_test,Xc_test,best_solution_test]=efficient_pnp(x3d_h,x2d_h_test,A);

%% Create Verification Data
alpha=-170/(180/pi);
beta=30/(180/pi);
gamma=-80/(180/pi);
R_ver1 = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
     sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
     -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma)];
for i=1:n
   intermediate_points = R_ver1*feature_points(:,i);
   ver_data(i,:) = (intermediate_points(1:2)/0.2037+256);
   ver_data_altered(i,1) = 0.5*ver_data(i,1)+128;
   ver_data_altered(i,2) = 0.5*ver_data(i,2)+128;
   ver_data_altered(i,:) = 1.1*10^-5*ver_data_altered(i,:);
end

x2d_h_ver = ver_data_altered;
x2d_h_ver(:,3) = 1;
[R_ver,T_ver,Xc_ver,best_solution_ver]=efficient_pnp(x3d_h,x2d_h_ver,A);
%% Use iterative method to correct for 2D/3D issues
R_new = R;
Xc_old = Xc;
Xc_diff = 11;
iteration = 0
T_new = T;
while sum(sum(Xc_diff)) > 1
    iteration = iteration +1
    for i=1:n
       X_cam(i,:) = R_new*feature_points(:,i);
       X_cam(i,3) = X_cam(i,3)+T_new(3);
       point(i).Ximg = project_3d_2d(A_new,X_cam(i,:)');
       x2d_hnew(i,:) = [point(i).Ximg(1) point(i).Ximg(2) 1];
    end

    [R_new,T_new,Xc_new,best_solution_new]=efficient_pnp(x3d_h,x2d_hnew,A);
    Xc_diff = Xc_new-Xc_old;
    Xc_old = Xc_new;
end