clear;
clc

data = table2array(readtable('features_data.txt'));

x1 = data(:,1);
y1 = data(:,2);
f1 = figure;
title('Visualisation of the 16 features in x and y over time')
xlabel('x') 
ylabel('y') 
for i = 1:16
    plot(data(:,i),data(:,i+1))
    hold on
    plot(data(1,i),data(1,i+1),'r*')
end
%legend('1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16')