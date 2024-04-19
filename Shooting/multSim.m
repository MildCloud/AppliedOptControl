function [tout, qout] = multSim(x, shotNum)
% input x is the decision vector

m = 1;
g = 9.8;
L = 1;

Nu = 1;
Nq = 2;
N = (numel(x)-1)/(Nu+Nq);

q0 = [x(N+shotNum);x(2*N+shotNum)];
% This finds the initial condition in the decision vector

u_sim = x(1:N); % all the input for the sim
u_shot = [x(shotNum) x(shotNum+1)]; % inputs for our shot

t_sim = linspace(0, x(end), N); % Series of time points
t_shot = [t_sim(shotNum) t_sim(shotNum+1)]; % time span of our shot

% use the sim instead of the shot for the interpolate, in case the
% simulation step outside the time interval(over shot)
% So the definition here is the same as the definition in simTrajectory.m
interp_torque = @(t) interp1(t_sim, u_sim, t);

% Define the dynamics for the pendulum with an applied torque
odefun = @(t,y) [y(2); -g/L*sin(y(1)) + interp_torque(t)/(m*L^2)];

% Simulates dynamics with ode45
% t_shot is the time interval for the simulation and q0 is the initial
% condition
% The difference between this function and the ode45 in the simTrajectory.m
% is that this function only simulate a time interval(shot) in the whole
% trajectory
[tout, qout] = ode45(odefun, t_shot, q0);