clear all
mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));

r = [0 ; 150 ; 0];
v = [0 ; 0 ; 0];
dq = [0.2302 ; 0.1434 ; 0.6358 ; -0.7226];
w = [-0.0873 ; -0.1489 ; 0.0262];
x = [r ; v ; dq ; w];


for i = 1:36

dr = [4-3*cos(n) 0 0 sin(n)/n 2*(1-cos(n))/n 0 ; 6*(sin(n)-n) 1 0 2*(cos(n)-1)/n (4*sin(n)-3*n)/n 0 ; 0 0 cos(n) 0 0 sin(n)/n];
dv = [3*n*sin(n) 0 0 cos(n) 2*sin(n) 0 ; 6*n*(cos(n)-1) 0 0 -2*sin(n) 4*cos(n)-3 0 ; 0 0 -n*sin(n) 0 0 cos(n)];

dq = eul2quat(w','XYZ');
q = [dq(1) -dq(2) -dq(3) -dq(4) ; dq(2) dq(1) dq(4) -dq(3) ; dq(3) -dq(4) dq(1) dq(2) ; dq(4) dq(3) -dq(2) dq(1)];
%q = [0 dq(3) -dq(2) dq(1) ; -dq(3) 0 dq(1) dq(2) ; dq(2) -dq(1) 0 dq(3) ; -dq(1) -dq(2) -dq(3) 0];
fi = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) q zeros(4,3) ; zeros(3,10) eye(3)];

x(:,i+1) = fi * x(:,i);


end

figure;
plot(x(7,:))
hold on
title("Propagation in quaternions")
plot(x(8,:))
plot(x(9,:))
plot(x(10,:))