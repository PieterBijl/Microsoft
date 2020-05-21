clear;
clc;

Eul_0_tar_RW = [-170;30;-80]; %deg (initial Euler angles)
w_0_tar_RW = [-0.0873;-0.1489;0.0262]; %rad/s (angular velocity)

data = table2array(readtable('features_data.txt'));

x1 = data(:,1);
y1 = data(:,2);
f1 = figure; 
for i = 1:16
    plot(data(:,i),data(:,i+1))
    hold on
    plot(data(2,i),data(2,i+1),'r*')
end
title('Visualisation of the 16 features in x and y over time')
xlabel('x') 
ylabel('y')
%legend('1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16')