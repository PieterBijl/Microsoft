% EPnP

clear all; close all;

addpath EPnP;

feature_data = importdata('features_data.txt'); %in pixels
timestep = 1;
feature_loc_pixels = feature_data(timestep,:);
n = size(feature_loc_pixels,2)/2;
for i = 1:n
    x2d_image(i, 1) = feature_loc_pixels(i*2-1);
    x2d_image(i, 2) = feature_loc_pixels(i*2);
    x2d_image(i, 3) = 1;
end

feature_points = 1/100*importdata('feature_points.txt'); %in meters
x3d_image = feature_points';
x3d_image(:,4) = 1;


width = 512; height = 512; % 512x512 pixels
u0 = 256; v0 = 256; % optical center
f = 3.9 * 10^-3; % focal length [m]
fx = 512; fy = 512;


n = 16;

x3d=zeros(n,4);
x2d=zeros(n,3); 
A = [fx 0 u0; 0 fy v0; 0 0 1];

[Rp,Tp,Xc,sol]=efficient_pnp(x3d_image,x2d_image,A);

