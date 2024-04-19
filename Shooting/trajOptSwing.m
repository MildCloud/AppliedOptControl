clc
clear
close all

N = 15; % number of points for the torque spline


costfcn = @(x) x(end);
x0 = [4 * ones(N, 1); 3];
% x0 is the initial guess decision vector: [N tau variables; tf]

% Set the optimization options
options = optimoptions('fmincon', 'Display', 'iter');
% f(x) is the cost function that we defined, at iter 0 f(x) = initial guess
% Feasibility is the measurement distance from the contraints, at iter 0 Fea = 3pi/4

xstar = fmincon(costfcn, x0, [], [], [], [], [-20 * ones(N, 1); 0], [20 * ones(N, 1); Inf], @nonlcon, options);
% Define the lower bound, upper bound and the nonlinear constraints
% xstar is the optimal decision vector, [N tau variables; tf]
% fmincon use gradient descent?
[tout, yout] = simTrajectory(xstar);

subplot(2, 2, 2)
plot(tout, yout(:, 1), 'k-')
xlabel('time (s)')
ylabel('angle (rad)')

subplot(2, 2, 4)
plot(linspace(1, tout(end), N), xstar(1:end-1))
xlabel('time (s)')
ylabel('torque (N*m)')