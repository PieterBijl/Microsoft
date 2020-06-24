% The State Transition Matrix F  [13 x 13]

%test_state = [-0.115163657344148,0.242879180561813,154.640374787836, 0, 0, 0, -0.701226945952288,0.646905610017730,0.219920983077943,0.203540323277840, 0, 0, 0];
%statepropagation(test_state, 1)

function f = state_transition1(state_vector,n)


delta_thetas = 2*pi/n;
omega_s = 0;

f = zeros(13,13);

f(1,[1:6]) = [1, 0, 6*(delta_thetas - sin(delta_thetas)), 1/(omega_s*(2*sin(delta_thetas)-3*delta_thetas)), 0, 2/(omega_s*(1-cos(delta_thetas)))];
f(2,[1:6]) = [0, cos(delta_thetas), 0, 0, 1/(omega_s*sin(delta_thetas)), 0];
f(3,[1:6]) = [0, 0, 4-3*cos(delta_thetas), 2/(omega_s*(cos(delta_thetas) -1)), 0, sin(delta_thetas)/omega_s];
f(4,[1:6]) = [0, 0, 6*omega_s*(1-cos(delta_thetas)), 4*cos(delta_thetas) - 3, 0, 2*sin(delta_thetas)];
f(5,[1:6]) = [0, omega_s*sin(delta_thetas), 0, 0, cos(delta_thetas), 0];
f(6,[1:6]) = [0, 0, 3*omega_s*sin(delta_thetas), -2*sin(delta_thetas), 0, cos(delta_thetas)];

f(7,7) = state_vector(7)+ dt*0.5*(state_vector(12)*state_vector(8) - state_vector(12)*state_vector(9) + state_vector(11)*state_vector(10)));
f(8,8) = state_vector(8) + dt*0.5*(-state_vector(13)*state_vector(7) + state_vector(11)*state_vector(9) + state_vector(12)*state_vector(10));
f(9,9) = state_vector(9) + dt*0.5*(state_vector(12)*state_vector(7) - state_vector(11)*state_vector(8) + state_vector(13)*state_vector(10));
f(10,10) = state_vector(10) + dt*0.5*(-state_vector(11)*state_vector(7) - state_vector(12)*state_vector(8) - state_vector(13)*state_vector(9));

f([11:13],[11:13]) = eye(3);
end
% 
% function F = statepropagation(state_vector, delta_t)
% 
% 
% 
% %f(7, [7,13]) = 
% 
% 
% 
% end