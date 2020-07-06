clear all

mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));

x0 = [0;0;150;0;0;0;-0.740005004767932;0.620430950681117;0.198931570392145;0.166985803757845;-0.0873;-0.1489;0.0262];
x(:,1) = x0;
t_sim = 100;
dt = 0.1;

for i = 1:(t_sim/dt)

dr = [4-3*cos(n) 0 0 sin(n)/n 2*(1-cos(n))/n 0 ; 6*(sin(n)-n) 1 0 2*(cos(n)-1)/n (4*sin(n)-3*n)/n 0 ; 0 0 cos(n) 0 0 sin(n)/n];
dv = [3*n*sin(n) 0 0 cos(n) 2*sin(n) 0 ; 6*n*(cos(n)-1) 0 0 -2*sin(n) 4*cos(n)-3 0 ; 0 0 -n*sin(n) 0 0 cos(n)];

%dq = 0.5*[0 x_1(13) -x_1(12) x_1(11) ; -x_1(13) 0 x_1(11) x_1(12) ; x_1(12) -x_1(11) 0 x_1(13) ; -x_1(11) -x_1(12) -x_1(13) 0];
%dq1 = 0.5*[-x_1(8) -x_1(9) -x_1(10) ; x_1(7) -x_1(10) x_1(9) ;  x_1(10) -x_1(7) -x_1(8) ; -x_1(9) x_1(8) x_1(7)];
%dq2 = 0.5*[x_1(7) -x_1(8) -x_1(9) ; x_1(8) x_1(7) -x_1(10) ;  x_1(9) x_1(10) x_1(7) ; x_1(10) -x_1(9) x_1(8)];
dqw = 0.5*[0 x(13,i) -x(12,i) x(11,i) ; -x(13,i) 0 x(11,i) x(12,i) ; x(12,i) -x(11,i) 0 x(13,i) ; -x(11,i) -x(12,i) -x(13,i) 0]*x(7:10,i);
dq3 = [dqw(1) -dqw(2) -dqw(3) -dqw(4) ; dqw(2) dqw(1) dqw(4) -dqw(3) ; dqw(3) -dqw(4) dqw(1) dqw(2) ; dqw(4) dqw(3) -dqw(2) dqw(1)];

%fi = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) dq zeros(4,3) ; zeros(3,10) eye(3)];
%fi1 = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) eye(4) dq1 ; zeros(3,10) eye(3)];
%fi2 = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) eye(4) dq2 ; zeros(3,10) eye(3)];
fi3 = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) dq3 zeros(4,3) ; zeros(3,10) eye(3)];


%x_k = [0;0;0;0;0;0;x_1(7:10);0;0;0] + fi*x_1;
%x_k = fi1*x_1;
%x_k = fi2*x_1;
x(:,i+1) = fi3*x(:,i);
end