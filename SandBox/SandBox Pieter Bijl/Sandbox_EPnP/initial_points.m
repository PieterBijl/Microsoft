%clear all;
%clc;

feature_data = importdata('features_data.txt'); %in pixels
feature_points = 1/100*importdata('feature_points.txt'); %in meters
n = 16; % Number of datapoints

% Initial angles in radians
alpha=-170/(180/pi);
beta=30/(180/pi);
gamma=-80/(180/pi);

pixel_size = 0.2037; %pixel in meters, this value has been calculated by comparing the initial state to the first measurement and converting the units

R = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
     sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
     -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma)];
 
datapoint = 1; % which measurement is being used (from 1 to 36)

%% Calculate the Centroid of the Feature Data
centroid_pix = [0 0]';
for i=1:36
    for j=1:n
        centroid_pix = centroid_pix + [feature_data(i,2*j-1); feature_data(i,2*j)]; % Verify that the true centroid is 256,256
    end
end 
centroid_pix = centroid_pix/(36*n);

%% Calculate all points
for i=1:n
   point(i).pix(1) = feature_data(datapoint,2*i-1);
   point(i).pix(2) = feature_data(datapoint,2*i);
   point(i).model(1) = feature_points(1,i);
   point(i).model(2) = feature_points(2,i);
   point(i).model(3) = feature_points(3,i);
   point(i).model_modified = R*feature_points(:,i);
   point(i).model_modified_pix = point(i).model_modified/pixel_size;
   model_modified_pix_true(i,:) = point(i).model_modified_pix(1:2)+centroid_pix;
end

%% Do Plotting
figure;
subplot(3,1,1);
title('measurement data')
hold on
for i=1:n
   plot(point(i).pix(1),point(i).pix(2),'.','color',[1 0 0]);
end
subplot(3,1,2);
title('wireframe model rotated with initial angles')
hold on;
for i=1:n
    plot(point(i).model_modified(1),point(i).model_modified(2),'.','color',[1 0 0]);
end
subplot(3,1,3);
title('wireframe model rotated in pixels')
hold on;
for i=1:n
    plot(model_modified_pix_true(i,1),model_modified_pix_true(i,2),'.','color',[1 0 0]);
end