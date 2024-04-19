function [C, Ceq] = nonlconmult(x)

% In multiple shooting, we will simulate multiple shots
% Then we compare to the next decision variable for state

Nu = 1;
Nq = 2;
N = (numel(x)-1)/(Nu+Nq);

Ceq = [];

% Shot Loop (multiple shooting)
for iter = 1:N-1
    % Simulate a shot
    [tout, qout] = multSim(x, iter);
    % final_state is the end state of the shot
    final_state = qout(end, :).';
    next_state = [x(N+iter+1); x(2*N+iter+1)];
    defect = next_state - final_state;
    Ceq = [Ceq; defect];
end

C = []; % no nonlinear inequality constraints

% Add task contraints or bounday constrants or path constraints
Ceq = [Ceq; [final_state(1) - pi; final_state(2)]];

% Add initial condition
Ceq = [Ceq; [x(N+1); x(2*N+1)]];
