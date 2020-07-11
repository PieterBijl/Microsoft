function [dH_dq]=dH_dq(q,xyz)
% This function takes the rotation matrix in quaternions and takes the
% derivative of it. It then multiplies this with the xyz coordinates in the
% camera frame of a feature and then puts this together to form a 3x4
% matrix.
dH_dq_1=[2*q(1) -2*q(4) 2*q(3);
         2*q(4) 2*q(1) -2*q(2);
         -2*q(3) 2*q(2) 2*q(1)]*xyz;
dH_dq_2=[2*q(2) 2*q(3) 2*q(4);
         2*q(3) -2*q(2) -2*q(1);
         2*q(4) 2*q(1) -2*q(2)]*xyz;
dH_dq_3=[-2*q(3) 2*q(2) 2*q(1);
         2*q(2) 2*q(3) 2*q(4);
         -2*q(1) 2*q(4) -2*q(3)]*xyz;
dH_dq_4=[-2*q(4) -2*q(1) 2*q(2);
         2*q(1) -2*q(4) 2*q(3);
         2*q(1) 2*q(3) 2*q(4)]*xyz;
dH_dq=[dH_dq_1 dH_dq_2 dH_dq_3 dH_dq_4];
end