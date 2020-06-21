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
fi_cw = [dr ; dv];

x_pos = [0 ; 0 ; 0 ; 0 ; 0 ; 0]; %initial?????? [r ; v]
x_pos = fi_cw*x_pos;

%kinematic Euler equations
w = [-0.0873 ; -0.1489 ; 0.0262]; %rad/s, given parameter: constant angular velocity
th_0 = [-170 ; 30 ; -80]; %deg, given parameter: initial Euler angles
th = th_0*pi/180;

dth = [1 sin(th(1))*tan(th(2)) cos(th(1))*tan(th(2)) ; 0 cos(th(1)) -sin(th(1)) ; 0 sin(th(1))/cos(th(2)) cos(th(1))/cos(th(2))] * w + n * [sin(th(3))/cos(th(2)) ; cos(th(3)) ; tan(th(2))*sin(th(3))]; 
th = th + dth*dt;

x_att = [th ; w];

%kinematic quaternion equations
q_1 = sin(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)-cos(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
q_2 = cos(th(1)/2)*sin(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*cos(th(2)/2)*sin(th(3)/2);
q_3 = cos(th(1)/2)*cos(th(2)/2)*sin(th(3)/2)-sin(th(1)/2)*sin(th(2)/2)*cos(th(3)/2); 
q_4 = cos(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
q = [q_1 ; q_2 ; q_3 ; q_4];

dq = [0 w(3) -w(2) w(1) ; -w(3) 0 w(1) w(2) ; w(2) -w(1) 0 w(3) ; -w(1) -w(2) -w(3) 0]/2 + n * [q(3) ; q(4) ; -q(1) ; -q(2)]/2;
q = q + dq*dt;

%state space matrix
x = [x_pos ; x_att];
