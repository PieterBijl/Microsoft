function [Xc, Tc, R, euler, quat] = EPnP(wireframe,datapoints)
% Inputs
% wireframe: wireframe in meters
% datapoints: datapoints of the measurement in pixels
% std_noise: standard noise in pixels that is added to the image
%
% Outputs
% Xc: the coordinates from all feature points as seen from the camera in
% meters
% Tc: the translation vector tc to go from the camera to the centre of mass
% of the target
% R: the rotation matrix to transform the wireframe to the current attitude
% of the target satellite
% euler: Euler angles that form R in radians
% quat: quaternions that form R
[x3d_h,x2d_h,A]=EPnP_data(wireframe,datapoints);
[R,Tc,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A);
euler=rotm2eul(R);
quat=[sin(euler(1)/2)*cos(euler(2)/2)*cos(euler(3)/2)-cos(euler(1)/2)*sin(euler(2)/2)*sin(euler(3)/2); 
      cos(euler(1)/2)*sin(euler(2)/2)*cos(euler(3)/2)+sin(euler(1)/2)*cos(euler(2)/2)*sin(euler(3)/2); 
      cos(euler(1)/2)*cos(euler(2)/2)*sin(euler(3)/2)-sin(euler(1)/2)*sin(euler(2)/2)*cos(euler(3)/2); 
      cos(euler(1)/2)*cos(euler(2)/2)*cos(euler(3)/2)+sin(euler(1)/2)*sin(euler(2)/2)*sin(euler(3)/2)]; %calcualte quaternions from euler angles
end