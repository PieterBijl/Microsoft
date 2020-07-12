clear all

load('x_real.mat')
x0 = x_real(:,1);

% x0(3) = 10;
t_end = 6000
x = zeros(13,t_end);
x(:,1) = x0;
for t=2:t_end
    [x(:,t),phi] = prediction(x(:,t-1),t);
end

figure;
plot(x(1,:))
hold on
plot(x(2,:))
plot(x_real(2,1:t_end))
plot(x(3,:))

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