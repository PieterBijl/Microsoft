function [R_quat_new]=R_quat_new(q)
R_quat_new= [1-2*(q(4)^2+q(3)^2) 2*(q(4)*q(1)-q(3)*q(2)) 2*(q(3)*q(1)+q(4)*q(2));
             2*(q(1)*q(4)+q(3)*q(2)) 1-2*(q(1)^2+q(3)^2) 2*(q(3)*q(4)-q(1)*q(2));
             2*(q(1)*q(3)-q(4)*q(2)) 2*(q(4)*q(3)+q(1)*q(2)) 1-2*(q(1)^2+q(4)^2)];
end