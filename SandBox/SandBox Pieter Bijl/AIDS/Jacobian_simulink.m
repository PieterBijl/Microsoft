function [h,H]= Jacobian_simulink(feature_points,x_k_1)
    
    %focal length of the camera
    fx = 2*3.9*10^-3;
    fy = 2*3.9*10^-3;
    
    %pixelsize
    pixel_size = 1.1*10^-5;
    
    %number of featurepoints
    n = 16;

    %individual vector definition
    vectorJacobian = zeros(2*n, 3);
    quaternionJacobian = zeros(2*n, 4);
    h = zeros(32,1);
    
    %import and convert featured data points
    wireframe = feature_points;
    
    for i = 1:n
        
        %quaternion rotation matrix
        q = x_k_1(7:10);
        R_q = [q(1)^2+q(2)^2-q(3)^2-q(3)^2 2*(q(2)*q(3)-q(1)*q(4)) 2*(q(4)*q(2)+q(1)*q(3));
         2*(q(4)*q(2)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2;
         2*(q(2)*q(3)+q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)-q(1)*q(2))];
        
        %position vector
        xyz = R_q*wireframe(:,i)+x_k_1(1:3);
        
        %h vector calculation
        h(2*i-1,1) = xyz(1)/xyz(3)*fx+256*pixel_size;
        h(2*i,1) = xyz(3)/xyz(3)*fy+256*pixel_size;
        
        %position Jacobian calculation
        vectorJacobian(i*2-1:i*2,1:3) = [fx/xyz(3) 0 -fx*xyz(1)/xyz(3)^2 ; 0 fy/xyz(3) -fy*xyz(2)/xyz(3)^2];
        
        %quaternion rotation Jacobian calculation
        dH_dq_1 = [2*q(1) -2*q(4) 2*q(3); 2*q(4) 2*q(1) -2*q(2); -2*q(3) 2*q(2) 2*q(1)]*xyz;
        dH_dq_2 = [2*q(2) 2*q(3) 2*q(4); 2*q(3) -2*q(2) -2*q(1); 2*q(4) 2*q(1) -2*q(2)]*xyz;
        dH_dq_3 = [-2*q(3) 2*q(2) 2*q(1); 2*q(2) 2*q(3) 2*q(4); -2*q(1) 2*q(4) -2*q(3)]*xyz;
        dH_dq_4 = [-2*q(4) -2*q(1) 2*q(2); 2*q(1) -2*q(4) 2*q(3); 2*q(1) 2*q(3) 2*q(4)]*xyz;
        dH_dq = [dH_dq_1 dH_dq_2 dH_dq_3 dH_dq_4] * [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 -1];
        
        quaternionJacobian(i*2-1:i*2,1:4) = vectorJacobian(i*2-1:i*2,1:3)*dH_dq;
        
    end
    
    %final Jacobian vector
    H = [vectorJacobian, zeros(2*n, 3), quaternionJacobian, zeros(2*n, 3)];

end

