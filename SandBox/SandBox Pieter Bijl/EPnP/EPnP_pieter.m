clc;
clear all;

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters

fx=354.5; %focal length in pixels
fy=354.5; 
f=3.9*10^-3; %focal length
pixel_size = 1.1*10^-5;

u0=256; v0=256;
width=512; height=512;
m = 1
A=[fx/m 0 u0/m 0; 0 fy/m v0/m 0; 0 0 1 0];
camera_points = feature_data(1,:); %in pixels

figure;
for i=1:16
    scatter(camera_points(1,2*i-1),camera_points(1,2*i))
    hold on
end