function [q,q_dot,q_ddot,t1] = Quintic_syms(q0,ta,jump)
q=0;
q_dot=0;
q_ddot=0;
a = length(q0);
t0 = 0;
tf = 0;
syms a1 b1 c1 d1 e1 f1 real
for i=1:a-1
    % t = ta(i):jump:ta(i+1);
    % tout = [tout t];
    t0 = ta(i);
    tf = ta(i+1);
    mat = [1 t0 t0^2 t0^3 t0^4 t0^5;...
        0 1 2*t0 3*(t0^2) 4*(t0^3) 5*(t0^4);...
        1 tf (tf^2) (tf^3) (tf^4) (tf^5);...
        0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);...
        0 0 2 6*t0 12*t0^2 20*t0^3;...
        0 0 2 6*tf 12*(tf^2) 20*(tf^3)];
    coeff1 = solve(mat*[a1;b1;c1;d1;e1;f1] == [q0(i);0;q0(i+1);0;0;0],[a1 b1 c1 d1 e1 f1]);
    syms t;
    dbl = t.*t;
    tree = dbl.*t;
    fr = tree.*t;
    fve = fr.*t;
    q = piecewise(t<t0 , q , (t>=t0)&(t<tf),(double([coeff1.a1 coeff1.b1 coeff1.c1 coeff1.d1 coeff1.e1 coeff1.f1])*[1;t;dbl;tree;fr;fve]));
    q_dot = piecewise(t<t0,q_dot ,(t>=t0)&(t<tf),(double([coeff1.a1 coeff1.b1 coeff1.c1 coeff1.d1 coeff1.e1 coeff1.f1])*[0;1;2*t;3*dbl;4*tree;5*fr]));
    q_ddot = piecewise(t<t0,q_ddot ,(t>=t0)&(t<tf),(double([coeff1.a1 coeff1.b1 coeff1.c1 coeff1.d1 coeff1.e1 coeff1.f1])*[0;0;2;6*t;12*dbl;20*tree]));
end

t1 = ta(1):jump:tf;

end