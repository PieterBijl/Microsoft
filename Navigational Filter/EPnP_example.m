clear all
clc

addpath EPnP;

feature_data = importdata('features_data_extended.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
measurement_number = 1;
n = 16;
u0 = 256;
v0 = 256;
fx = 2*3.9*10^-3;
fy = fx;
m = 1.1*10^-5;
A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];

x = zeros(14,length(feature_data));
for i=1:length(feature_data)
    x3d_h=zeros(n,4);
x2d_h=zeros(n,3); 
    for j=1:n
        x3d_h(j,1:3) = feature_points(:,j)';
        x3d_h(j,4) = 1;
        x2d_h(j,1) = feature_data(i,2*j-1);
        x2d_h(j,2) = feature_data(i,2*j);
        x2d_h(j,3) = 1;
    end
    x(1,i) = i;
    x(2:14,i) = EPnP2state_vector(x3d_h,x2d_h,A);
end
