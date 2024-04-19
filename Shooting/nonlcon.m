function [C, Ceq] = nonlcon(x)
% x is the decision vector
m = 1;
g = 9.8;
L = 1;
odefun_torque = @(t, y) [y(2); -g/L*sin(y(1)) + interp1(linspace(0, x(end), numel(x) - 1), x(1:end - 1), t)/ (m * L^2)]; % feed forward
[tout, yout] = ode45(odefun_torque, [0 x(end)], [0; 0]);

final_state = yout(end, :).';

C = [];
Ceq = [final_state(1) - pi, final_state(2)];