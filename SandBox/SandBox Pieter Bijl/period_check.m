omega_1 = -0.0873;
omega_2 = -0.1489;
omega_3 = 0.0262;

t_1 = abs(2*pi/omega_1);
t_2 = abs(2*pi/omega_2);
t_3 = abs(2*pi/omega_3);

f_1 = 1/t_1;
f_2 = 1/t_2;
f_3 = 1/t_3;

syms t
S = vpasolve(sin(2*pi*f_1*t)==sin(2*pi*f_2*t)==sin(2*pi*f_3*t)==0, t, 40)

%check if it is zero
for i=1:9000
    time(i) = i;
    x(i,1) = sin(2*pi*f_1*i);
    x(i,2) = sin(2*pi*f_2*i);
    x(i,3) = sin(2*pi*f_3*i);
end

plot(time,x(:,1))
hold on
plot(time,x(:,2))
plot(time,x(:,3))