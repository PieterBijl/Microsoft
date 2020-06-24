% The Jacobian Matrix H with dimension [2n x 13]

%test_state = [-0.115163657344148,0.242879180561813,154.640374787836, 0, 0, 0, -0.701226945952288,0.646905610017730,0.219920983077943,0.203540323277840, 0, 0, 0];
%Jacobian_test = Jacobian1(16, test_state, 3.9e-3, 3.9e-3)

function J = Jacobian(n,state_vector, fx, fy,wireframe)
    vectorJacobian = zeros(2*n, 3);
    quaternionJacobian = zeros(2*n, 3);
    for i = 1:n
        H = H_int(fx, fy, wireframe(1,i)+state_vector(1), wireframe(2,i)+state_vector(2), wireframe(3,i)+state_vector(3));
        vectorJacobian(i*2-1:i*2,1:3) = H;
        q = state_vector(7:10);
        H_q = H*H_ext(q);
        quaternionJacobian(i*2-1:i*2,1:3) = H_q;
    end
    J = [vectorJacobian, zeros(2*n, 3), quaternionJacobian, zeros(2*n, 3)];
end