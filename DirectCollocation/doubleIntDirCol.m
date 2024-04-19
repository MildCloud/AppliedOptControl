clc
clear
close all

N = 10;

costFcn = @(x) x(end);
options = optimoptions('fmincon', 'display', 'iter', 'maxfunevals', 1e4, 'specifyobjectivegradient', false, 'specifyconstraintgradient', true);

tic
xstar = fmincon(costFcn, ones(3*N+1, 1), [], [], [], [], [], [], @nonlcon, options);
toc

subplot(2, 1, 1)
plot(linspace(0,xstar(end),N), xstar(1:N), 'k-')
xlabel('time (s)')
ylabel('Force (N)')

subplot(2, 1, 2)
plot(linspace(0,xstar(end),N), xstar(N+1:2*N), 'k-')
xlabel('time (s)')
ylabel('Diatance (m)')