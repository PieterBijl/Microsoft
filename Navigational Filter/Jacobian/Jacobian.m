% The Jacobian Matrix H with dimension [2n x 13]

%test_state = [-0.115163657344148,0.242879180561813,154.640374787836, 0, 0, 0, -0.701226945952288,0.646905610017730,0.219920983077943,0.203540323277840, 0, 0, 0];
%Jacobian_test = Jacobian1(16, test_state, 3.9e-3, 3.9e-3)

function J = Jacobian(n,state_vector, fx, fy,wireframe)
% This function gives back the Jacobian and takes as input the state_vector
% for timestep t, n the number of featurepoints, fx and fy as the focal
% length of the camera and the wireframe in meters. It first calculates the
% rotation matrix so that it can transform all featurepoints into the
% camera frame. After that it calculates the derivative with respect to
% position and the derivative of quaternions wrt quaternions
    vectorJacobian = zeros(2*n, 3);
    quaternionJacobian = zeros(2*n, 4);
    for i = 1:n
        q = state_vector(7:10);
        xyz = R_quat(q)*wireframe(:,i)+state_vector(1:3);
        H = H_int(fx, fy, xyz(1), xyz(2), xyz(3));
        vectorJacobian(i*2-1:i*2,1:3) = H;
        H_q = H*dH_dq(q,xyz);
        quaternionJacobian(i*2-1:i*2,1:4) = H_q;
    end
    J = [vectorJacobian, zeros(2*n, 3), quaternionJacobian, zeros(2*n, 3)];
end