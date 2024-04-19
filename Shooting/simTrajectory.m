function [tout, yout] = simTrajectory(x)
% input x is the decision vector

m = 1;
g = 9.8;
L = 1;

% Created a function that interpolates the torque for the current time, t,
% from the discretized times and torques defined by the decision vector, x
% x = [u1; ...un; tf]
% linspace(0, x(end), numel(x) - 1) contains numel(x) - 1 of t value that
% generates corresponding x(1:end-1) output.
interp_torque = @(t) interp1(linspace(0, x(end), numel(x) - 1), x(1:end-1), t);

% Define the dynamics for the pendulum with an applied torque
odefun = @(t,y) [y(2); -g/L*sin(y(1)) + interp_torque(t)/(m*L^2)];

% Simulates dynamics with ode45
[tout, yout] = ode45(odefun, [0 x(end)], [0; 0]);
% tout, yout is the same as the tout, yout in the nonlcon

% DEBUG
% disp('I am simulating')
