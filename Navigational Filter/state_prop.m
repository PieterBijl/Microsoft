function x_1 = state_prop(x_0,t,dt)
    % this functions propagates the state and takes a state vector x0 as
    % input (13x1), t is the time in seconds after the start of
    % measurements/initialization, dt is the preferred timestep in seconds,
    % which will most likely be 1 second.
    mu = 3.986*10^14;
    R_earth = 6371000;
    H_target = 773000;
    a = R_earth + H_target;
    n = sqrt(mu/(a^3));
    t=1;
    phi = [4-3*cos(n*t) 0 0 1/n*sin(n*t) 2/n*(1-cos(n*t)) 0;
       6*(sin(n*t)-n*t) 1 0 2/n*(cos(n*t)-1) 1/n*(4*sin(n*t)-3*n*t) 0;
       0 0 cos(n*t) 0 0 1/n*sin(n*t);
       3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0;
       6*n*(cos(n*t)-1) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0;
       0 0 -n*sin(n*t) 0 0 cos(n*t)];
   q_dot = eul2quat(x_0(11:13)'*dt,'XYZ'); % This is the unit quaternion rotation due to omega
   x_1(1:6,1) = phi*x_0(1:6);
   x_1(7:10,1) = quatnormalize(quatmultiply(x_0(7:10)',q_dot))';
   x_1(11:13,1) = x_0(11:13);
end