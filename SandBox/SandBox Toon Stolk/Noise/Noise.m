% Noise.m
% Combines both the functions NoiseBoolean and NoiseStd to make a noise
% estimation for the navigational filter.

n = 1000;   % Number of points for the plot

% Create data for plotting
angles = linspace(0, 2*pi, n);
bools = ones(1, n);
stds = ones(1, n);

for i=1:n
    bools(i) = NoiseBoolean(angles(i), (1/4)*2*pi, (1/12)*2*pi);
    stds(i) = NoiseStd(angles(i), 1, 6);
end
noise = bools.*stds;

% Plot the noise Boolean, Magnitude and Estimation
figure
hold on
plot(angles/pi*180, bools,'--')
plot(angles/pi*180, stds,'--')
plot(angles/pi*180, noise)% '--', '--', '-')
ylim([0 7])
xlim([0 360])

ylabel("Noise Standard Deviation [pixels]")
xlabel("Illumination Angle [\psi]")

legend("Noise Boolean", "Noise Magnitude", "Noise Estimation", "Location", "Northwest")
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
