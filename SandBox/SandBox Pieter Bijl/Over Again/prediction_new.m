function [x_k_1,fi]= prediction_new(x_1,t)

%x_k_1 = x_1;

%orbital parameter
mu = 3.986*10^14;
R_earth = 6371000;
H_target = 773000;
a = R_earth + H_target;
n = sqrt(mu/(a^3));
om = n; % argument of perigree

%clohessy-wiltshire equations
dr = [1 0 6*(n-sin(n)) 1/om*(4*sin(n)-3*n) 0 2/om*(1-cos(n));
      0 cos(n) 0 0 1/om*sin(n) 0;
      0 0 4-3*cos(n) 2/om*(cos(n)-1) 0 sin(n)/om];
dv = [0 0 6*om*(1-cos(n)) 4*cos(n)-3 0 2*sin(n);
      0 om*sin(n) 0 0 cos(n) 0;
      0 0 3*om*sin(n) -2*sin(n) 0 cos(n)];
% dr = [4-3*cos(n*t) 0 0 sin(n*t)/n 2*(1-cos(n*t))/n 0 ; 6*(sin(n*t)-n*t) 1 0 2*(cos(n*t)-1)/n (4*sin(n*t)-3*n*t)/n 0 ; 0 0 cos(n*t) 0 0 sin(n*t)/n];
% dv = [3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0 ; 6*n*(cos(n*t)-1) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0 ; 0 0 -n*sin(n*t) 0 0 cos(n*t)];

%quaternion kinematic equation
%dq = 0.5*[0 x_1(13) -x_1(12) x_1(11) ; -x_1(13) 0 x_1(11) x_1(12) ; x_1(12) -x_1(11) 0 x_1(13) ; -x_1(11) -x_1(12) -x_1(13) 0];
wq = eul2quat(x_1(11:13)','XYZ'); % This is the unit quaternion rotation due to omega
dq = [wq(1) -wq(2) -wq(3) -wq(4) ; wq(2) wq(1) wq(4) -wq(3) ; wq(3) -wq(4) wq(1) wq(2) ; wq(4) wq(3) -wq(2) wq(1)];
%x_k_1(1:6) = phi*x_1(1:6);
%x_k_1(7:10) = quatnormalize(quatmultiply(x_1(7:10)',dq1))';
%state transition matrix
fi = [dr zeros(3,7) ; dv zeros(3,7) ; zeros(4,6) dq zeros(4,3) ; zeros(3,10) eye(3)];

%propogation step
%x_k_1 = [0;0;0;0;0;0;x_1(7:10);0;0;0] + fi * x_1;
x_k_1 = fi * x_1;
x_k_1(7:10) = quatnormalize(x_k_1(7:10)')';
% no_corr_out = fi * no_corr_in;
end