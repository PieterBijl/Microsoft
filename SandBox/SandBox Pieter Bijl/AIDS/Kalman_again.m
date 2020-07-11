clear all
close all

addpath Jacobian

%% Get the data
load('euler_real.mat')
load('x_real.mat')
feature_data = importdata('features_data_extended.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

%% Camera data
n = 16;
u0 = 256;
v0 = 256;
fx = 2*3.9*10^-3;
fy = fx;
m = 1.1*10^-5;
A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];

%% Pose initialisation
std_noise = 1;

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
x0 = [Tp(1); Tp(3); Tp(2); 0.0001; -0.001; 0.0001; q(1); q(2); q(3); q(4); -0.0853; -0.1429; 0.0212];

%% Kalman Propagation
t_end = 6000;
R = std_noise*eye(32,32);
% for i = 1:31
%     R(i,i+1) = 0.01*randn(1,1);
%     R(i+1,i) = 0.01*randn(1,1);
% end
x_diff = abs(x0-x_real(:,1));
p0 = 10*abs(x_diff);%x_diff.*x_diff;
% p0(1:3) = 10*p0(1:3);
p = diag(p0');
% p = 0.1*diag([1,10,1,0,0,0,0.1,0.1,0.1,0.1,0.01,0.01,0.01]);
Q = 10^-6*diag([1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1]);%*eye(13,13);
x = zeros(13,t_end);
x(:,1) = x0;
plot_on = 1;

for i=1:t_end-1
    [x_p_1(:,i),phi] = prediction_test(x(:,i));
    p_k = phi * p * phi' + Q;
    z(:,i) = m*(feature_data(i+1,:)'+std_noise*randn(32,1));
    H = Jacobian(n,x_p_1(:,i), fx, fy,feature_points);
    h(:,i) = observation_model(x_p_1(:,i),feature_points);

    %kalman gain calculation
    K = p_k * H' * inv(H * p_k * H' + R);

    %correction step
    y(:,i) = z(:,i) - h(:,i);
    x_k(:,i) = K*y(:,i);
    x(:,i+1) = x_p_1(:,i) + x_k(:,i);
%     eul_ver(:,i+1) = rad2deg(quat2eul(x(7:10,i)'))';
    p = (eye(13,13) - K*H)*p_k;
end

if plot_on ==1
    %% Plotting Quaternions
    figure;
    subplot(2,2,1)
    plot(x(7,:))
    hold on
    plot(x_real(7,1:t_end))
    subplot(2,2,2)
    plot(x(8,:))
    hold on
    plot(x_real(8,1:t_end))
    subplot(2,2,3)
    plot(x(9,:))
    hold on
    plot(x_real(9,1:t_end))
    subplot(2,2,4)
    plot(x(10,:))
    hold on
    plot(x_real(10,1:t_end))

    %% Plotting Translation
    figure;
    plot(x(1,:))
    hold on
    plot(x_real(1,1:t_end))
    plot(x(2,:))
    plot(x_real(2,1:t_end))
    plot(x(3,:))
    plot(x_real(3,1:t_end))
end