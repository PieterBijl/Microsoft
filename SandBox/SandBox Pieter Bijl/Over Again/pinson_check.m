clear all
load('measured_state_xyz_quatfixed.mat')
load('x_real.mat')

for i=1:6012
    measurement_matrix_full_orbit(2:13,i) = x(2:13,i+3);
    measurement_matrix_full_orbit(1,i) = i;
end