
mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));

euler_initial = deg2rad([-180 30 -80]);
quat_initial = eul2quat(euler_initial,'ZYX');
%x0 = [0;150;0;0;0;0;quat_initial';0*-0.0862;    -0.1496;   0.000];
x0 = [0;150;0;0;0;0;quat_initial';-0.0873;-0.1489;0.0262];
x(:,1) = x0;
euler(:,1) = quat2eul(x(7:10,1)','ZYX')';
t_sim = 200;
dt = 1;
t = 0;
for i =1:(t_sim/dt)
phi = [4-3*cos(n*t) 0 0 1/n*sin(n*t) 2/n*(1-cos(n*t)) 0;
       6*(sin(n*t)-n*t) 1 0 2/n*(cos(n*t)-1) 1/n*(4*sin(n*t)-3*n*t) 0;
       0 0 cos(n*t) 0 0 1/n*sin(n*t);
       3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0;
       6*n*(cos(n*t)-1) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0;
       0 0 -n*sin(n*t) 0 0 cos(n*t)];
q_dot = eul2quat(x(11:13,i)'*dt,'XYZ');%0.5*[0 x(11,i) -x(12,i) x(11,i) ; -x(13,i) 0 x(11,i) x(12,i) ; x(12,i) -x(11,i) 0 x(13,i) ; -x(11,i) -x(12,i) -x(13,i) 0]*x(7:10,i)*dt;%n/2*[x(9,i); x(10,i); -x(7,i); -x(8,i)];
%q_dot = 0.5*[0 x(11,i) -x(12,i) x(11,i) ; -x(13,i) 0 x(11,i) x(12,i) ; x(12,i) -x(11,i) 0 x(13,i) ; -x(11,i) -x(12,i) -x(13,i) 0]*x(7:10,i)*dt;%n/2*[x(9,i); x(10,i); -x(7,i); -x(8,i)];
t = t + dt;
x(1:6,i+1) = phi*x(1:6,i);
q_t(i,:) = quatmultiply(x(7:10,i)',q_dot)';
x(7:10,i+1) = quatnormalize(q_t(i,:))';
euler(:,i+1) = quat2eul(x(7:10,i+1)')';
x(11:13,i+1) = x(11:13,i);
end
euler = rad2deg(euler);
figure;
plot(x(7,:))
hold on
title("Propagation in quaternions")
plot(x(8,:))
plot(x(9,:))
plot(x(10,:))

%% Euler
figure;
plot(euler(1,:))
hold on
title("Propagation in Euler angles")
plot(euler(2,:))
plot(euler(3,:))
plot(euler_test(1,1:200))
plot(euler_test(2,1:200))
plot(euler_test(3,1:200))