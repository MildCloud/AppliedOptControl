function M = Mfunc(t,in2)
%Mfunc
%    M = Mfunc(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    11-Nov-2023 11:40:03

th1_ = in2(1,:);
th2_ = in2(2,:);
t2 = cos(th1_);
t3 = sin(th1_);
t4 = th1_+th2_;
t5 = cos(t4);
t6 = sin(t4);
t7 = t2./2.0;
t8 = t3./2.0;
t9 = t5./2.0;
t10 = t5./4.0;
t11 = t6./2.0;
t12 = t6./4.0;
t13 = t2+t9;
t14 = t3+t11;
t15 = t7+t10;
t16 = t8+t12;
t17 = t10.*t13;
t18 = t12.*t14;
t19 = t9.*t15;
t20 = t11.*t16;
t21 = t17+t18+t19+t20+1.0;
M = reshape([t13.*t15.*2.0+t14.*t16.*2.0+t2.^2./4.0+t3.^2./4.0+2.0,t21,t21,t5.*t10+t6.*t12+1.0],[2,2]);
end
