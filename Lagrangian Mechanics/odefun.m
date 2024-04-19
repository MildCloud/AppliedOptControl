function [dq] = odefun(t, q)

th1 = q(1);
dth1 = q(2);
th2 = q(3);
dth2 = q(4);

ddth = Mfunc(t,[th1;th2])^-1*ffunc(t,[th1;th2],[dth1;dth2]);

dq = zeros(4,1);
dq(1) = q(2);
dq(2) = ddth(1);
dq(3) = q(4);
dq(4) = ddth(2);

end

