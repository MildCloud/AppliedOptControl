% runMPC.m

% 1. Sample the current state
% 2. Run a Trajectory Optimization
% 3. Use the optimal control input for Ts seconds
% 4. to 1

clc
clear
close all

% Define our parameters
T = 1; % Finite time horizon (Used for TO)
q0 = [0;0;0;0];
qdes = [1;0;0;0];
Q = eye(4); % Sate cost
R = eye(2); % Input cost
Fmax = 10;

% Simulation and MPC Control Loop
% quick simulation test

Tfinal = 10;
Ts = 0.1;

t_sim = 0:Ts:Tfinal;

t_all = [];
q_all = [];

for iter = 1:numel(t_sim)
    % Step 1: Sample the current state
    % It's q0 at the beginning
    
    % Step 2: Formulate and run the trajectory optimization
    H = Hfunc(Q, R, qdes, T);
    c = cfunc(Q, R, qdes, T);
    A = Afunc(T, Fmax);
    b = bfunc(T, Fmax);
    Aeq = Aeqfunc(q0, T, Fmax);
    beq = beqfunc(q0, T, Fmax);
    
    xstar = quadprog(H, c, A, b, Aeq, beq);

    % Step 3: Apply the control inuts for Ts seconds
    N = 21;
    u1 = xstar(1:2:N*2);
    u2 = xstar(2:2:N*2);

    odefunparams = @(t,q) odefun(t,q,[interp1(linspace(0, T, N), u1, t);interp1(linspace(0, T, N), u2, T)]);
    [tout, qout] = ode45(odefunparams, [0 Ts], q0);

    t_all = [t_all; tout+t_sim(iter)];
    q_all = [q_all; qout];
    % Debug plot
    % plot(tout, qout(:, 1))
    % pause

    % Sample the current state;
    q0 = qout(end,:).';
end
subplot(2, 1, 1)
plot(t_all, q_all(:, 1))
xlabel('time')
ylabel('x pos')
subplot(2, 1, 2)
plot(t_all, q_all(:, 2))
xlabel('time')
ylabel('y pos')