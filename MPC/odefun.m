function [dq] = odefun(t, q, u)

% Dynamics
m = 1;
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; 1/m 0; 0 0; 0 1/m];

dq = A*q + B*u;