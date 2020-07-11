clear all

addpath EPnP

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
std_noise = 2;
for n_reduction = 1:12
    for i = 1:6000
        x3d_h=zeros(n-n_reduction,4);
        x2d_h=zeros(n-n_reduction,3);
        y = sort(datasample(1:16,16-n_reduction,'Replace',false));
        wireframe_reduced = zeros(3,16-n_reduction);
        feature_data_reduced = zeros(32-2*n_reduction,1);
        for k = 1:length(y)
            wireframe_reduced(:,k) = feature_points(:,y(k));
            feature_data_reduced(2*k-1) = feature_data(i,2*y(k)-1);
            feature_data_reduced(2*k) = feature_data(i,2*y(k));
        end
        for j=1:n-n_reduction
            x3d_h(j,1:3) = wireframe_reduced(:,j)';
            x3d_h(j,4) = 1;
            x2d_h(j,1) = feature_data_reduced(2*j-1,1)+randn(1,1)*std_noise;
            x2d_h(j,2) = feature_data_reduced(2*j,1)+randn(1,1)*std_noise;
            x2d_h(j,3) = 1;
        end
        [Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
        q = rotm2quat(Rp);
        e_t_1(n_reduction,i) = abs(Tp(1)-x_real(1,i));
        e_t_2(n_reduction,i) = abs(Tp(2)-x_real(3,i)); % The y value of the EPnP corresponds to the z value of the filter
        e_t_3(n_reduction,i) = abs(Tp(3)-x_real(2,i)); % The z value of the EPnP corresponds to the y value of the filter
        beta = quatmultiply(x_real(7:10,i)',q);
        e_q(n_reduction,i) = 2*acos(abs(beta(1)));
    end
    e_t_m_1(n_reduction) = mean(e_t_1(n_reduction,:));
    e_t_m_2(n_reduction) = mean(e_t_2(n_reduction,:));
    e_t_m_3(n_reduction) = mean(e_t_3(n_reduction,:));
    e_q_m(n_reduction) = abs(mean(e_q(n_reduction,:)));
end

%% Plotting
figure;
subplot(2,2,1)
plot(e_t_m_1)
hold on
title("EPnP x Error")
xlabel("Number of removed feature points")
ylabel("Error in meters")
subplot(2,2,2)
plot(e_t_m_2)
title("EPnP y Error")
xlabel("Number of removed feature points")
ylabel("Error in meters")
subplot(2,2,3)
plot(e_t_m_3)
title("EPnP z Error")
xlabel("Number of removed feature points")
ylabel("Error in meters")
subplot(2,2,4)
plot(e_q_m)
title("EPnP Euler Axis-Angle Error")
xlabel("Number of removed feature points")
ylabel("Attitude Error in Degrees")
