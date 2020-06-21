%% dynamical model state space matrix 

%timestep
t = 1;

%target orbital rate
mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));

%Clohessy-Wiltshire (CW) equations
drr = [4-3*cos(n*t) 0 0 ; 6*(sin(n*t)-n*t) 1 0 ; 0 0 cos(n*t)];
drv = [sin(n*t)/n 2*(1-cos(n*t))/n 0 ; 2*(cos(n*t)-1)/n (4*sin(n*t)-3*n*t)/n 0 ; 0 0 sin(n*t)/n];
dvr = [3*n*sin(n*t) 0 0 ; 6*n*(cos(n*t)-1) 0 0 ; 0 0 -n*sin(n*t)];
dvv = [cos(n*t) 2*sin(n*t) 0 ; -2*sin(n*t) 4*cos(n*t)-3 0 ; 0 0 cos(n*t)]; 

dr = [drr drv];
dv = [dvr dvv];

x_cw = [dr ; dv];

%kinematic quaternions equations
dq = [0 om(3) -om(2) om(1) ; -om(3) 0 om(1) om(2) ; om(2) -om(1) 0 om(3) ; -om(1) -om(2) -om(3) 0]/2 + n*[q(3) ; q(4) ; -q(1) ; -q(2)]/2;


