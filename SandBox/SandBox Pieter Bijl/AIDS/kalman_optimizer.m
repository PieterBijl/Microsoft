function f = kalman_optimizer(v_t)
    addpath Jacobian

    feature_data = importdata('features_data_extended.txt'); %in pixels
    feature_points = 1/100*importdata('feature_points.txt'); %in meters
    measurement_number = 1;

    n = 16;
    u0 = 256;
    v0 = 256;
    fx = 2*3.9*10^-3;
    fy = fx;
    m = 1.1*10^-5;
    A=[fx/m 0 u0; 0 fy/m v0; 0 0 1];
    t_end = 1000;
    std_noise = 0;

    x3d_h=zeros(n,4);
    x2d_h=zeros(n,3); 
    for j=1:n
        x3d_h(j,1:3) = feature_points(:,j)';
        x3d_h(j,4) = 1;
        x2d_h(j,1) = feature_data(measurement_number,2*j-1)+randn(1,1)*std_noise;
        x2d_h(j,2) = feature_data(measurement_number,2*j)+randn(1,1)*std_noise;
        x2d_h(j,3) = 1;
    end
    [Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
    q = rotm2quat(Rp);
    eul_ver = rad2deg(quat2eul(q));
    % x0 = [Tp(1); Tp(3); Tp(2); 0.001; 0.001; 0.001; q(1); q(2); q(3); q(4); -0.0873; -0.1489; 0.0262];
    x0 = [Tp(1); Tp(3); Tp(2); 0.001; 0.001; 0.001; q(1); q(2); q(3); q(4); -0.0873; -0.1489; 0.0262];
    R = eye(32,32)*2*m;
    Q = 0;
    v = 1/10*[v_t(1), v_t(2), v_t(3), 0,0,0,0.5,0.5,0.5,0.5,0.1,0.1,0.1];
    % p = diag(v);
    %p = 0.1*diag([10,100,10,0,0,0,0.5,0.5,0.5,0.5,0.1,0.1,0.1]);
    p = diag(v);
    x(:,1) = x0;
    % eul_ver = zeros(3,t_end);
    % eul_ver(:,1) = rad2deg(quat2eul(x(7:10,1)'))';

    %% Get the 'real values'
    load('euler_real.mat')
    load('x_real.mat')

    %% Do the calculations
    e_t = zeros(3,t_end); % error matrix for the three translations
    e_q = zeros(4,t_end); % error matrix for the four quaternions
    e_t(:,1) = abs(x_real(1:3,1)-x(1:3,1));
    e_q(:,1) = quatmultiply(x_real(7:10,1)',x(7:10,1)')';
    for i=1:t_end-1
        [x_k_1(:,i),p_p,phi] = prediction(x(:,i),Q,p,i);

        z = m*feature_data(measurement_number+1,:)';
        H = Jacobian(n,x_k_1(:,i), fx, fy,feature_points);
        h = observation_model(x_k_1(:,i),feature_points);

        %kalman gain calculation
        K = p_p * H' * inv(H * p_p * H' + R);

        %correction step
        x(:,i+1) = x_k_1(:,i) + K*(z - h);
        e_t(:,i+1) = abs(x_real(1:3,i+1)-x(1:3,i+1));
%         e_q(:,i+1) = quatmultiply(x_real(7:10,i+1)',x(7:10,i+1)')';
    %     eul_ver(:,i+1) = rad2deg(quat2eul(x(7:10,i)'))';
        p = (eye(13,13) - K*H)*p_p;
    end

    %% Change the data
    eul_ver = zeros(3,t_end);
    for i=1:length(x)
        eul_ver(:,i) = rad2deg(quat2eul(x(7:10,i)'))';
    end

    %% Find the complete error
    e_t_total = sum(sum(e_t));
    e_d = zeros(3,t_end); % Create matrix for euler angle errors
    for i=1:t_end
       e_d(:,i) = abs(euler_real(:,i)-eul_ver(:,i));
    end
    e_d_total = sum(sum(e_d));
    f = e_t_total;
end