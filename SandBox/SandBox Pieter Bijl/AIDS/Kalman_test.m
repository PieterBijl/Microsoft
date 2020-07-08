clear all

addpath Jacobian

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
measurement_number = 1;

n = 16;
u0 = 256;
v0 = 256;
fx = 2*3.9*10^-3;
fy = fx;
m = 1.1*10^-5;
A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];

x3d_h=zeros(n,4);
x2d_h=zeros(n,3); 
for j=1:n
    x3d_h(j,1:3) = feature_points(:,j)';
    x3d_h(j,4) = 1;
    x2d_h(j,1) = feature_data(measurement_number,2*j-1);
    x2d_h(j,2) = feature_data(measurement_number,2*j);
    x2d_h(j,3) = 1;
end
[Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
q = rotm2quat(Rp);
eul_ver = rad2deg(quat2eul(q));
x = [Tp(1); Tp(3); Tp(2); 0.001; 0.001; 0.001; q(1); q(2); q(3); q(4); -0.0873; -0.1489; 0.0262];
Q = 0;
p_1 = eye(13,13);
[x_k_1,p_k_1,phi] = prediction(x,Q,p_1,1);
eul_ver_2 = rad2deg(quat2eul(x_k_1(7:10)'));

%% Jacobian
[h,H]= Jacobian_simulink(feature_points,x_k_1);
z = m*feature_data(measurement_number+1,:)';
R_q = [q(1)^2+q(2)^2-q(3)^2-q(3)^2 2*(q(2)*q(3)-q(1)*q(4)) 2*(q(4)*q(2)+q(1)*q(3));
         2*(q(4)*q(2)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2;
         2*(q(2)*q(3)+q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)-q(1)*q(2))];
H_bijl = Jacobian(n,x_k_1, fx, fy,feature_points);
h_bijl = observation_model(x_k_1,feature_points);

%kalman gain calculation
R = eye(32,32)*2*m;
K = p_k_1 * H_bijl' * inv(H_bijl * p_k_1 * H_bijl' + R);

%correction step
x_k = x_k_1 + K*(z - h_bijl);
p_k = (eye(13,13) - K*H)*p_k_1;