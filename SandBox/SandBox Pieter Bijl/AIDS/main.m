clear all
%% first
v0 = [10,100,10];
first_check = kalman_optimizer(v0)
%%
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
A = []
b = []
Aeq = [];
beq = [];
lb = [0 0 0];
ub = [];
nonlcon = [];

[v,fval] = fmincon(@kalman_optimizer,v0,A,b,Aeq,beq,lb,ub,nonlcon,options)
%%
check = kalman_optimizer(v)

%% perform with GA
options = optimoptions('ga','Display','iter','MaxGenerations',1000,'PlotFcn', @gaplotbestf,'CrossoverFraction', 0.8)
x_test = ga(@kalman_optimizer,13,A,b,Aeq,beq,lb,ub,nonlcon,options)

%% 
tic;
kalman_optimizer(v0)
toc;