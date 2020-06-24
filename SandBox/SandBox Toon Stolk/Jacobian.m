% The Jacobian Matrix H with dimension [2n x 13]

function J = Jacobian(state_vector)
    n = length(state_vector)
    
    vectorJacobian = zeros(2*n, 3);
    for i = 1:n
        H = H_int(400, 400, state_vector(1), state_vector(2), state_vector(3));
        vectorJacobian(i*2-1:i*2,1:3) = H;
    end
    
    quaternionJacobian = zeros(2*n, 4);
    for i = 1:2*n
        
    end

    J = [vectorJacobian, zeros(2*n, 3), ones(2*n, 4) zeros(2*n, 3)];
end