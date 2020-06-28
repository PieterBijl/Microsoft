%w = [-0.0873;-0.1489;0.0262];
w = [0.0262; -0.1489; -0.0873];
w_deg = rad2deg(w);
theta(:,1) = deg2rad([-180 30 -80]);
dt = 0.001;
n = 0.01;
for i=1:(100/dt)
    theta_dot(:,i) = (1/cos(theta(2,i))) * [cos(theta(2,i)) sin(theta(1,i))*sin(theta(2,i)) cos(theta(1,i))*sin(theta(2,i)) ; 0 cos(theta(1,i))*cos(theta(2,i)) -sin(theta(1,i))*cos(theta(2,i)) ; 0 sin(theta(1,i)) cos(theta(1,i))] * w+(n/cos(theta(2,i))) * [sin(theta(3,i)) ; cos(theta(2,i))*cos(theta(3,i)) ; sin(theta(2,i))*sin(theta(3,i))];
    theta(:,i+1) = theta(:,i)+theta_dot(:,i)*dt;
end
theta_deg = rad2deg(theta);
figure
plot(theta_deg(1,:))
hold on
plot(theta_deg(2,:))
plot(theta_deg(3,:))