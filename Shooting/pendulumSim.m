clc
clear
close all

m = 1;
g = 9.8;
L = 1;
tau = 10;

% state: [theta, dtheta] ^ T

% The ODE defined here is using the first order form convention, i.e. 
% dy/dt = f(y(t), t)
odefun = @(t, y) [y(2); -g/L*sin(y(1))];
odefun_torque = @(t, y) [y(2); -g/L*sin(y(1)) + tau / (m * L^2)]; % feed forward
odefun_torque_b = @(t, y) [y(2); -g/L*sin(y(1)) + (10 * (pi - y(1))+2*(0-y(2)))/ (m * L^2)]; % feedback

% Step of the trajectory starting from the initial condition
tspan = [0 10];

% The initial state vector
y0 = [pi/4; 0];

[tout, yout] = ode45(odefun_torque_b, tspan, y0);
% yout is the state vector starting from the initial y0 and to the tspan,
% which contains total size(tout) element; tout is the time for the
% corresponding state vectors

plot(tout, yout(:, 1), 'k-')
xlabel('time (s)')
ylabel('angle (rad)')