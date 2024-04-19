clc
clear
close all


% Run a quick test optimization
T = 1;
q0 = [0;0;0;0];
qdes = [1;0;0;0];
Q = eye(4);
R = eye(2);
Fmax = 10;

H = Hfunc(Q, R, qdes, T)
c = cfunc(Q, R, qdes, T)
A = Afunc(T, Fmax)
b = bfunc(T, Fmax)
Aeq = Aeqfunc(q0, T, Fmax)
beq = beqfunc(q0, T, Fmax)

xstar = quadprog(H, c, A, b, Aeq, beq)