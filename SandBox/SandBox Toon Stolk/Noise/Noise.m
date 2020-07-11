
n = 6012;

angles = linspace(0, 2*pi, n);
bools = ones(1, n);
stds = ones(1, n);
for i=1:n
    bools(i) = NoiseBoolean(angles(i), 0.1*pi, 0.3*pi);
    stds(i) = NoiseStd(angles(i), 1, 6);
end
noise = bools.*stds;

plot(angles/pi*180, bools, angles/pi*180, stds, angles/pi*180, noise)
xlim([0 360])
% n0 = 2;
% n = zeros(1,361);
% for i = 1:180
%     if 85<i <95
%         n(i) = 10;
%     else
%         n(i) = n0 / cos(i/180*pi);
%     end
%     
% end
% 
% plot(n)