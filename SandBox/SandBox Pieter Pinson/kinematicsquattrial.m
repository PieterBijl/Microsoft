close all;
%q = [-0.7226, 0.6358, 0.1434, 0.2302];
q1 = [-0.7226 0.6358 0.1434 0.2302];
w = [-0.0873;-0.1489;0.0262];
for i = 1:36 
    %q(:,i+1) = cross(q(:,i),quatrotate(q(:,i)*w));
    qw(:,i) = q1(i,:)*[0 w(3) -w(2) w(1) ; -w(3) 0 w(1) w(2) ; w(2) -w(1) 0 w(3) ; -w(1) -w(2) -w(3) 0]/2;
    q1(:,i+1) = quatmultiply(q1(:,i),qw(:,i));
end


plot(q)