% LinPend.m

clc
clear
close all

m = 1;
g = 9.8;
L = 1;


tspan = [0 10];
q0 = [99*pi/100; 0];
% as q0 close to q_star, the lin simulation is better

% Define fixed point:
q_star = [pi;0];
u_star = 0;

% Linear simulation of out pendulum
A = [0 1; -g/L*cos(q_star(1)) 0];
B = [0; 1/(m*L^2)];

Q = eye(2);
R = 1;
K = lqr(A, B, Q, R);

u = 0;
odelin = @(t, q) A*(q-q_star) + B*(u - u_star);

odefun = @(t,q) [q(2); -g/L*sin(q(1)) + -K*(q-q_star)/m*L^2];

% Nonlinear simulation of our pendulum
[tout, qout] = ode45(odefun, tspan, q0);

% Simulation of the linearized system
[tlin, qlin] = ode45(odelin, tspan, q0);

plot(tout, qout(:, 1), 'k--') %, tlin, qlin(:, 1), 'r-', 'LineWidth', 3)