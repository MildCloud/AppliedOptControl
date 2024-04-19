function [C, Ceq, jc, jceq] = nonlcon(x)

% Constants
Fmax = 10;
m = 1;
sf = 5;

% Grab all of our inputs and states

N = (numel(x)-1)/3;
u = x(1:N);
% u(end) is not used here
s = x(N+1:2*N);
ds = x(2*N+1:3*N);
T = x(end);

% inequality constraints
C = [u-Fmax; -u-Fmax; -T];

% equality constraints
Ceq = [s(1); ds(1); s(end)-sf; ds(end)];
Ceq = [Ceq; s(2:end)-s(1:end-1)-ds(1:end-1)*(T/(N-1))];
Ceq = [Ceq; ds(2:end)-ds(1:end-1)-(u(1:end-1)/m)*(T/(N-1))];

jc = JC.';
jceq = JCeq(x).';