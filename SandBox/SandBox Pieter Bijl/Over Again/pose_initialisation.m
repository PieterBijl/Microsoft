clear all

feature_data = importdata('features_data_extended.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

%% Perform EPnP calculation
% Load in camera data
n = 16;
u0 = 256;
v0 = 256;
fx = 2*3.9*10^-3;
fy = fx;
m = 1.1*10^-5;
A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];
std_noise = 2;
x3d_h=zeros(n,4);
x2d_h=zeros(n,3); 

for j=1:n
    x3d_h(j,1:3) = feature_points(:,j)';
    x3d_h(j,4) = 1;
    x2d_h(j,1) = feature_data(1,2*j-1)+std_noise*randn(1,1);
    x2d_h(j,2) = feature_data(1,2*j)+std_noise*randn(1,1);
    x2d_h(j,3) = 1;
end
[Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
x(1:3) = Tp;
x(7:10) = rotm2quat(Rp);
x(11:13) = [-0.0873;-0.1489;0.0262];
for i=1:13
    round(x(i),4)
end