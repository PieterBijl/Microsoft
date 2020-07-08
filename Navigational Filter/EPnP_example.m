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

%% Featuredata
feature_data_extended = zeros(33,length(feature_data));
for i=1:length(feature_data)
    feature_data_extended(1,i) = i;
    feature_data_extended(2:33,i) = feature_data(i,:)';
end

%% Euler angle bonanza
euler_test = zeros(3,length(feature_data));
for i=1:length(feature_data)
   euler_test(:,i) = round(rad2deg(quat2eul(x(8:11,i)'))); 
   x(8:11,i) = eul2quat(deg2rad(euler_test(:,i)'));
end


%%
figure;
plot(x(8,1:100));
hold on
plot(x(9,1:100));
plot(x(10,1:100));
plot(x(11,1:100));

%%
x_new = x(:,4:6012);
for i=1:length(x_new)
    x_new(1,i) = i;
end
