function p_norm = norm(p_k)
    
    q = [p_k(7,7) ; p_k(8,8) ; p_k(9,9) ; p_k(10,10)];
    
    qrot = [q(2)^2+q(3)^2+q(4)^2 , -q(1)*q(2), -q(1)*q(3), -q(1)*q(4) ; -q(2)*q(1), q(1)^2+q(3)^2+q(4)^2, -q(2)*q(3), -q(2)*q(4) ; -q(3)*q(1), -q(3)*q(2), q(1)^2+q(2)^2+q(4)^2, -q(3)*q(4) ; -q(4)*q(1), -q(4)*q(2), -q(4)*q(3), q(1)^2+q(2)^2+q(3)^2];

    dqnorm = (q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2)^(-3/2) * qrot;

    J = [eye(6) zeros(6,7) ; zeros(4,6) dqnorm zeros(4,3); zeros(3,10) eye(3)];
    
    p_norm = J * p_k * J';

end
