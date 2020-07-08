clear all
v0 = 0.1*[10,100,10,0,0,0,0.5,0.5,0.5,0.5,0.1,0.1,0.1];
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
A = []
b = []
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = [];
[v,fval] = fmincon(@kalman_optimizer,v0,A,b,Aeq,beq,lb,ub,nonlcon,options)

%% 
tic;
kalman_optimizer(v0)
toc;