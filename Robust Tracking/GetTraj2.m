function [x1,x2,x3,y1,y2,y3,z1,z2,z3] = GetTraj2(t)

t2 = t*t;
t3 = t2*t;
t4 = t3*t;
t5 = t4*t;

if t<5
    x1=0;
    x2=0;
    x3=0;
elseif t<15
    d0 = 0;
    df = 1;
    t0 = 5;
    tf = 15;
    dt = tf-t0;
    b = 2*pi/dt;
    A = 9*b*(df-d0)/(8*dt);
    c = b*t0;
    x3 = A*sin(b*t-c);
    x2 = -1*(A/b)*cos(b*t-c) + (A/b);
    x1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0;
    b = 3*b;
    A = A/3;
    c = b*t0;
    x3 = x3 - A*sin(b*t-c);
    x2 = x2 + (A/b)*cos(b*t-c) - (A/b);
    x1 = x1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0);
elseif t<25
    x1 = 1;
    x2 = 0;
    x3 = 0;
elseif t<35
    d0 = 1;
    df = 0;
    t0 = 25;
    tf = 35;
    dt = tf-t0;
    b = 2*pi/dt;
    A = 9*b*(df-d0)/(8*dt);
    c = b*t0;
    x3 = A*sin(b*t-c);
    x2 = -1*(A/b)*cos(b*t-c) + (A/b);
    x1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0;
    b = 3*b;
    A = A/3;
    c = b*t0;
    x3 = x3 - A*sin(b*t-c);
    x2 = x2 + (A/b)*cos(b*t-c) - (A/b);
    x1 = x1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0);
else
    x1 = 0;
    x2 = 0;
    x3 = 0;
end

if t<15
    y1=0;
    y2=0;
    y3=0;
elseif t<25
    d0 = 0;
    df = 1;
    t0 = 15;
    tf = 25;
    dt = tf-t0;
    b = 2*pi/dt;
    A = 9*b*(df-d0)/(8*dt);
    c = b*t0;
    y3 = A*sin(b*t-c);
    y2 = -1*(A/b)*cos(b*t-c) + (A/b);
    y1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0;
    b = 3*b;
    A = A/3;
    c = b*t0;
    y3 = y3 - A*sin(b*t-c);
    y2 = y2 + (A/b)*cos(b*t-c) - (A/b);
    y1 = y1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0);
elseif t<35
    y1 = 1;
    y2 = 0;
    y3 = 0;
elseif t<45
    d0 = 1;
    df = 0;
    t0 = 35;
    tf = 45;
    dt = tf-t0;
    b = 2*pi/dt;
    A = 9*b*(df-d0)/(8*dt);
    c = b*t0;
    y3 = A*sin(b*t-c);
    y2 = -1*(A/b)*cos(b*t-c) + (A/b);
    y1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0;
    b = 3*b;
    A = A/3;
    c = b*t0;
    y3 = y3 - A*sin(b*t-c);
    y2 = y2 + (A/b)*cos(b*t-c) - (A/b);
    y1 = y1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0);
else
    y1 = 0;
    y2 = 0;
    y3 = 0;
end

if t<5
    z1 = (6*t5)/3125 - (3*t4)/125 + (2*t3)/25;
    z2 = (6*t4)/625 - (12*t3)/125 + (6*t2)/25;
    z3 = (24*t3)/625 - (36*t2)/125 + (12*t)/25;
elseif t<45
    z1 = 1;
    z2 = 0;
    z3 = 0;
elseif t<65
    z1 = - (8854437155380585*t5)/4722366482869645213696 + (33*t4)/64000 - (359*t3)/6400 + (3861*t2)/1280 - (41067*t)/512 + 432809/512;
    z2 = - (44272185776902925*t4)/4722366482869645213696 + (33*t3)/16000 - (1077*t2)/6400 + (3861*t)/640 - 41067/512;
    z3 = - (44272185776902925*t3)/1180591620717411303424 + (99*t2)/16000 - (1077*t)/3200 + 3861/640;
else
    z1 = 0;
    z2 = 0;
    z3 = 0;
end

end