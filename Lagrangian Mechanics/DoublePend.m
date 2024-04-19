% DoublePend.m

clc
clear
close all

syms t th1(t) th2(t)

m1 = 1;
m2 = 1;
L1 = 1;
L2 = 1;
J1 = 1; % why
J2 = 1; % should be a function of theta2?
g = 9.8;

% 1. define generalized coordinates
th = [th1; th2];

% 2. define the kinematics
r1 = [0.5*L1*sin(th1); -0.5*L1*cos(th1)];
r2 = [L1*sin(th1)+0.5*L2*sin(th1+th2); -L1*cos(th1)-0.5*L2*cos(th1+th2)];
ph1 = -pi/2+th1;
ph2 = -pi/2+th1+th2;

% 3. Calculate velocities
dr1 = diff(r1, t);
dr2 = diff(r2, t);

om1 = diff(ph1, t);
om2 = diff(ph2, t);

% 4. Sum kinetic energies
T = 0.5*m1*(dr1.')*dr1 + 0.5*m2*(dr2.')*dr2 + 0.5*J1*om1^2 + 0.5*J2*om2^2;
V = m1*g*[0 1]*r1 + m2*g*[0 1]*r2;

% 5. Compute the Lagrangian and the EL Equation
L = T - V;

dL_d_dth = [diff(L, diff(th1, t)); diff(L, diff(th2, t))];

% First term
d_dt_L = diff(dL_d_dth,t);

% Second term
dL_dth = [diff(L,th1); diff(L,th2)];

Qnc = 0;

% LHS of the EL equation (h(th, dth, ddth, tau))
EL_eq = d_dt_L - dL_dth - Qnc;

% 6. Rewrite in a common form

% Substitutions to eliminate 'diff' and time dependence t for Jacobian
syms ddth1_ ddth2_ dth1_ dth2_ th1_ th2_
EL_eq = subs(EL_eq, {diff(th1,t,t), diff(th2,t,t)}, {ddth1_, ddth2_});
EL_eq = subs(EL_eq, {diff(th1,t), diff(th2,t)}, {dth1_, dth2_});
EL_eq = subs(EL_eq, {th1, th2}, {th1_, th2_})

M = jacobian(EL_eq, [ddth1_; ddth2_])
f = -subs(EL_eq,{ddth1_,ddth2_},{0,0})

% 7. Solve for accelerations
% matlabFunction(M, 'File','Mfunc.m', 'Vars',{t, [th1_; th2_]})
% matlabFunction(f, 'File','ffunc.m', 'Vars',{t,[th1_; th2_],[dth1_;dth2_]})

% Simulation
[tout, qout] = ode45(@odefun, [0 10],[pi/4;0;pi/4;0])
plot(tout, qout(:,1))

% Animation
FPS = 20; % Fram rate in frames per second

t_anim = 0:1/FPS:max(tout);
% Interpolate angles
th1_anim = interp1(tout, qout(:,1), t_anim);
th2_anim = interp1(tout, qout(:,3), t_anim);

plot(t_anim, th1_anim, 'k.', t_anim, th2_anim, 'r.')

v = VideoWriter('doublePend.mp4', 'MPEG-4');
open(v)

figure
for iter = 1:numel(t_anim)
    p1 = L1*[sin(th1_anim(iter)); -cos(th1_anim(iter))];
    p2 = p1 + L2*[sin(th1_anim(iter) + th2_anim(iter)); -cos(th1_anim(iter) + th2_anim(iter))];
    plot([0 p1(1) p2(1)], [0 p1(2) p2(2)], 'k-')
    axis equal
    axis([-2.5 2.5 -2.5 2.5])
    drawnow
    frame = getframe(gcf);
    writeVideo(v, frame);
end
close(v)