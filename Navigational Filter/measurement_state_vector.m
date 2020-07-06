clear all
clc

addpath EPnP;
addpath Jacobian

std_noise = 0;
feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
wireframe = feature_points;
w = [-0.0873;-0.1489;0.0262]; 
for i=1:length(feature_data)
    datapoints = z_calc(feature_data(i,:),16,std_noise);
    [Xc, Tc, R, euler, quat] = EPnP(wireframe,datapoints);
    x_EPnP(:,i) = [Tc; zeros(3,1); quat; w];
    x_EPnP_Euler_deg(:,i) = [Tc; zeros(3,1); rad2deg(euler'); w];
end
figure
plot(x_EPnP(7,:))
hold on
title("measurements in quaternions")
plot(x_EPnP(8,:))
plot(x_EPnP(9,:))
plot(x_EPnP(10,:))

figure
plot(x_EPnP_Euler_deg(7,:))
hold on
title("measurements in Euler angles")
plot(x_EPnP_Euler_deg(8,:))
plot(x_EPnP_Euler_deg(9,:))

%% find omega
q1 = x_EPnP(7:10,2)
q0 = x_EPnP(7:10,1)

q_omega_dt = quatdivide(q0',q1')
omega = quat2eul(q_omega_dt,'XYZ')