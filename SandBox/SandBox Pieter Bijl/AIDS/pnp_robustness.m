clear all

%% Get the data
load('x_real.mat')
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

for std_noise = 1:6
    for i = 1:6000
        x3d_h=zeros(n,4);
        x2d_h=zeros(n,3); 
        for j=1:n
            x3d_h(j,1:3) = feature_points(:,j)';
            x3d_h(j,4) = 1;
            x2d_h(j,1) = feature_data(1,2*j-1)+randn(1,1)*std_noise;
            x2d_h(j,2) = feature_data(1,2*j)+randn(1,1)*std_noise;
            x2d_h(j,3) = 1;
        end
        [Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
        q = rotm2quat(Rp);
        e_t_1(std_noise,i) = abs(Tp(1)-x_real(1,i));
        e_t_2(std_noise,i) = abs(Tp(2)-x_real(3,i)); % The y value of the EPnP corresponds to the z value of the filter
        e_t_3(std_noise,i) = abs(Tp(3)-x_real(2,i)); % The z value of the EPnP corresponds to the y value of the filter
        beta = quatmultiply(x_real(7:10,i)',q);
        e_q(std_noise,i) = 2*acos(beta(1));
    end
    e_t_m_1(std_noise) = mean(e_t_1(std_noise,:));
    e_t_m_2(std_noise) = mean(e_t_2(std_noise,:));
    e_t_m_3(std_noise) = mean(e_t_3(std_noise,:));
    e_q_m(std_noise) = mean(e_q(std_noise,:));
end

%% Plotting
figure;
subplot(2,2,1)
plot(e_t_m_1)
hold on
title("EPnP x Error")
xlabel("Standard noise in pixels")
ylabel("Error in meters")
subplot(2,2,2)
plot(e_t_m_2)
title("EPnP y Error")
xlabel("Standard noise in pixels")
ylabel("Error in meters")
subplot(2,2,3)
plot(e_t_m_3)
title("EPnP z Error")
xlabel("Standard noise in pixels")
ylabel("Error in meters")
subplot(2,2,4)
plot(e_q_m)
title("EPnP Quaternion Error")
xlabel("Standard noise in pixels")
ylabel("Attitude Error")

