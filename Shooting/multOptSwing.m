clc
clear
close all

% number of discrete time points (Nodes)
N = 10;
tau_lim = 20;

costfcn = @(x) x(end);
x0 = [ones(3*N, 1); 4];
% x0 is the initial guess decision vector: [N tau variables; tf]
% f(x) is the cost function that we defined, at iter 0 f(x) = initial guess
% Feasibility is the measurement distance from the contraints, at iter 0 Fea = 3pi/4

xstar = fmincon(costfcn, x0, [], [], [], [], [-tau_lim*ones(N,1);-Inf(2*N, 1);0], [tau_lim*ones(N,1);Inf(2*N, 1);Inf], @nonlconmult);
% Define the lower bound, upper bound and the nonlinear constraints
% xstar is the optimal decision vector, [N tau variables; tf]
xstar

subplot(2, 1, 1)
title('control inputs')
plot(xstar(1:N))
subplot(2, 1, 2)
title('angle')
plot(xstar(N+1: 2*N))