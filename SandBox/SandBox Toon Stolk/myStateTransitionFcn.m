% Copyright 2018 The MathWorks, Inc. 

function x = myStateTransitionFcn(x,u)
% Sample time [s]
dt = 0.01; 

% Using Euler discretization, next states
% can be calculated given the current
% states and input 
x = x + [x(2); -9.81/0.5*sin(x(1)) + u]*dt;
end

