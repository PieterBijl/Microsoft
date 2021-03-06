clear all
clc

addpath EPnP;

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

%x(:,1) = [-0.0019; 0.0017; 151.0891; 0; 0; 0; 0.1670; 0.1989; 0.6204; -0.7400 ;-0.0873; -0.1489; 0.0262];

for measurement_number = 1:36
n = 16;
u0 = 256;
v0 = 256;
fx = 2*3.9*10^-3;
fy = fx;
m = 1.1*10^-5;
A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];
x3d_h=zeros(n,4);
x2d_h=zeros(n,3); 
for i=1:n
    x3d_h(i,1:3) = feature_points(:,i)';
    x3d_h(i,4) = 1;
    x2d_h(i,1) = feature_data(measurement_number,2*i-1);
    x2d_h(i,2) = feature_data(measurement_number,2*i);
    x2d_h(i,3) = 1;
end

x(:,measurement_number) = EPnP2state_vector(x3d_h,x2d_h,A);
end