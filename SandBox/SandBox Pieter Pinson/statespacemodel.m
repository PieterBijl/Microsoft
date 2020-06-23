%% dynamical model Envisat
close all;
clear all;
clc;

%timestep
dt = 1;

%target orbital rate
mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));

%initialization
w = [-0.0873 ; -0.1489 ; 0.0262]; %rad/s, given parameter: constant angular velocity
th_0 = [-170 ; 30 ; -80]; %deg, given parameter: initial Euler angles
th = th_0*pi/180; %rad
x_pos = [-0.0019 ; 0.0017 ; 1.0891 ; 0 ; 0 ; 0]; %m, initial position taken from measurements [r ; v]

x_att = [th ; w];
x = [x_pos ; x_att];

%quaternions
q_1 = sin(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)-cos(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
q_2 = cos(th(1)/2)*sin(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*cos(th(2)/2)*sin(th(3)/2);
q_3 = cos(th(1)/2)*cos(th(2)/2)*sin(th(3)/2)-sin(th(1)/2)*sin(th(2)/2)*cos(th(3)/2); 
q_4 = cos(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
q = [q_1 ; q_2 ; q_3 ; q_4];
qc1 = [q_1 ; q_2 ; q_3 ; q_4];

for i = 1:6031
    
%Clohessy-Wiltshire (CW) equations
drr = [4-3*cos(n*dt) 0 0 ; 6*(sin(n)-n*dt) 1 0 ; 0 0 cos(n*dt)];
drv = [sin(n*dt)/n 2*(1-cos(n*dt))/n 0 ; 2*(cos(n*dt)-1)/n (4*sin(n*dt)-3*n*dt)/n 0 ; 0 0 sin(n*dt)/n];
dvr = [3*n*sin(n*dt) 0 0 ; 6*n*(cos(n*dt)-1) 0 0 ; 0 0 -n*sin(n*dt)];
dvv = [cos(n*dt) 2*sin(n*dt) 0 ; -2*sin(n*dt) 4*cos(n*dt)-3 0 ; 0 0 cos(n*dt)]; 

dr = [drr drv];
dv = [dvr dvv];
fi_cw = [dr ; dv];

x_pos(:,i+1) = fi_cw*x_pos(:,i);

%kinematic Euler equations
dth(:,i) = [1 sin(th(1,i))*tan(th(2,i)) cos(th(1,i))*tan(th(2,i)) ; 0 cos(th(1,i)) -sin(th(1,i)) ; 0 sin(th(1,i))/cos(th(2,i)) cos(th(1,i))/cos(th(2,i))] * w; %+ n * [sin(th(3,i))/cos(th(2,i)) ; cos(th(3,i)) ; tan(th(2,i))*sin(th(3,i))]; 
th(:,i+1) = th(:,i) + dth(:,i)*dt;

x_att(:,i+1) = [th(:,i+1) ; w];

%state space matrix
x(:,i+1) = [x_pos(:,i+1) ; x_att(:,i+1)];

%Euler to quaternions
thq_1(i) = sin(th(1,i)/2)*cos(th(2,i)/2)*cos(th(3,i)/2)-cos(th(1,i)/2)*sin(th(2,i)/2)*sin(th(3,i)/2);
thq_2(i) = cos(th(1,i)/2)*sin(th(2,i)/2)*cos(th(3,i)/2)+sin(th(1,i)/2)*cos(th(2,i)/2)*sin(th(3,i)/2);
thq_3(i) = cos(th(1,i)/2)*cos(th(2,i)/2)*sin(th(3,i)/2)-sin(th(1,i)/2)*sin(th(2,i)/2)*cos(th(3,i)/2); 
thq_4(i) = cos(th(1,i)/2)*cos(th(2,i)/2)*cos(th(3,i)/2)+sin(th(1,i)/2)*sin(th(2,i)/2)*sin(th(3,i)/2);

thq(:,i) = [thq_1(i) ; thq_2(i) ; thq_3(i) ; thq_4(i)];

%kinematic quaternion equations
dq(:,i) =  [0 w(3) -w(2) w(1) ; -w(3) 0 w(1) w(2) ; w(2) -w(1) 0 w(3) ; -w(1) -w(2) -w(3) 0]*q(:,i)/2;% + n * [q(3,i) ; q(4,i) ; -q(1,i) ; -q(2,i)]/2;
q(:,i+1) = q(:,i) + dq(:,i)*dt;

dqc1(:,i) =  [qc1(4,i) -qc1(3,i) qc1(2,i) qc1(1,i) ; qc1(3,i) qc1(4,i) -qc1(1,i) qc1(2,i) ; -qc1(2,i) qc1(1,i) qc1(4,i) qc1(3,i) ; -qc1(1,i) -qc1(2,i) -qc1(3,i) qc1(4,i)]*[w ; 0]/2 + n * [q(3,i) ; q(4,i) ; -q(1,i) ; -q(2,i)]/2;
qc1(:,i+1) = qc1(:,i) + dqc1(:,i)*dt;

end

%graphs
figure; plot(x(1,:)); hold on; plot(x(2,:)); plot(x(3,:)); legend('x','y','z'); 
figure; plot(x(7,:)); hold on; plot(x(8,:)); plot(x(9,:)); legend('roll','pitch','yaw'); 
figure; plot(thq(1,:)); hold on; plot(thq(2,:)); plot(thq(3,:)); plot(thq(4,:)); legend('thq1','thq2','thq3','thq4'); 
figure; plot(q(1,:)); hold on; plot(q(2,:)); plot(q(3,:)); plot(q(4,:)); legend('q1','q2','q3','q4'); 
figure; plot(qc1(1,:)); hold on; plot(qc1(2,:)); plot(qc1(3,:)); plot(qc1(4,:)); legend('qc1','qc2','qc3','qc4'); 


%kinematic quaternion equations
%q_1 = sin(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)-cos(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
%q_2 = cos(th(1)/2)*sin(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*cos(th(2)/2)*sin(th(3)/2);
%q_3 = cos(th(1)/2)*cos(th(2)/2)*sin(th(3)/2)-sin(th(1)/2)*sin(th(2)/2)*cos(th(3)/2); 
%q_4 = cos(th(1)/2)*cos(th(2)/2)*cos(th(3)/2)+sin(th(1)/2)*sin(th(2)/2)*sin(th(3)/2);
%q = [q_1 ; q_2 ; q_3 ; q_4];

%dq = [0 w(3) -w(2) w(1) ; -w(3) 0 w(1) w(2) ; w(2) -w(1) 0 w(3) ; -w(1) -w(2) -w(3) 0]/2 + n * [q(3) ; q(4) ; -q(1) ; -q(2)]/2;
%q = q + dq*dt;

%state space matrix
%x = [x_pos ; x_att];

%simulink
%dth = [1 sin(th(1))*tan(th(2)) cos(th(1))*tan(th(2)) ; 0 cos(th(1)) -sin(th(1)) ; 0 sin(th(1))/cos(th(2)) cos(th(1))/cos(th(2))] * w + n * [sin(th(3))/cos(th(2)) ; cos(th(3)) ; tan(th(2))*sin(th(3))]; 
%drr = [4-3*cos(n*t) 0 0 ; 6*(sin(n*t)-n*t) 1 0 ; 0 0 cos(n*t)];
%drv = [sin(n*t)/n 2*(1-cos(n*t))/n 0 ; 2*(cos(n*t)-1)/n (4*sin(n*t)-3*n*t)/n 0 ; 0 0 sin(n*t)/n];
%dvr = [3*n*sin(n*t) 0 0 ; 6*n*(cos(n*t)-1) 0 0 ; 0 0 -n*sin(n*t)];
%dvv = [cos(n*t) 2*sin(n*t) 0 ; -2*sin(n*t) 4*cos(n*t)-3 0 ; 0 0 cos(n*t)]; 


%dq(:,i) =  [0 w(3) -w(2) w(1) ; -w(3) 0 w(1) w(2) ; w(2) -w(1) 0 w(3) ; -w(1) -w(2) -w(3) 0]*q(:,i)/2 + n * [q(3,i) ; q(4,i) ; -q(1,i) ; -q(2,i)]/2;
%q(:,i+1) = q(:,i) + dq(:,i)*dt;

