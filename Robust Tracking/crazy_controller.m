% Crazy_fly system
% Vishrut Bohara
% ID - 234400666
% alias - vbohara
% function equivalent to the state-space representation of RRBot
function [dxdt, u, wa] = crazy_controller(t1,x,K,lmbda)

%system values
m = 27*1e-3;
l = 46*1e-3;
Ix = 16.571710*1e-6;
Iy = 16.571710*1e-6;
Iz = 29.261652*1e-6;
Ip = 12.65625*1e-8;
KF = 1.28192*1e-8;
KM = 5.964552*1e-3;
g = 9.81;

ak = 1/(4*KF);
bk = sqrt(2)*ak/l;

Alloc = [ak,    -bk,    -bk,    -ak/KM;...
        ak,     -bk,    bk,     ak/KM;...
        ak,     bk,     bk,     -ak/KM;...
        ak,     bk,     -bk,    ak/KM];

% Trajectories--------------
[xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot] = GetTraj(t1);


phid2 = 0;
phid3 = 0;
ttad1 = 0;
ttad2 = 0;
ttad3 = 0;

x=num2cell(x);
[x1, y1, z1, phi1, tta1, shi1, x2, y2, z2, phi2, tta2, shi2] = deal(x{:});
omg_cap = 0;
rho = 20;

% desired control
Kp = 30;
Kd = 11;
ex1 = xd - x1;
ex2 = xd_dot - x2;
ey1 = yd - y1;
ey2 = yd_dot - y2;
uxd = xd_ddot + Kp*ex1 + Kd*ex2;
uyd = yd_ddot + Kp*ey1 + Kd*ey2;

% Controller----------------
%1
ez1 = zd - z1;
ez2 = zd_dot - z2;
s1 = ez2 + lmbda(1)*ez1;
u1 = m*(lmbda(1)*ez2 + zd_ddot + g + K(1)*sat(s1,0.1))/(cos(phi1)*cos(tta1));

x3 = (1/m)*(cos(phi1)*sin(tta1)*cos(shi1) + sin(phi1)*sin(shi1))*u1;
y3 = (1/m)*(cos(phi1)*sin(tta1)*sin(shi1) - sin(phi1)*cos(shi1))*u1;
ex3 = xd_ddot - x3;
ey3 = xd_ddot - y3;
duxd = Kp*ex2 + Kd*ex3;
duyd = Kp*ey2 + Kd*ey3;

%2 3
if(u1~=0)
    Fxd = m*uxd/u1;
    Fyd = m*uyd/u1;
    dFxd = m*duxd/u1;
    dFyd = m*duyd/u1;
    dFxd = 0;
    dFyd = 0;
else
    Fxd = 0;
    Fyd = 0;
    dFxd = 0;
    dFyd = 0;
end
phid1 = -asin(Fyd);
phid2 = -1*dFyd/cos(phi1);
ephi1 = phid1 - phi1;
ephi2 = phid2 - phi2;
s2 = ephi2 + lmbda(2)*ephi1;
vr2 = (Ip*rho*abs(tta2) + K(2)*Ix)*sat(s2,0.1);
u2 = phid3 + vr2 + Ix*lmbda(2)*ephi2 - tta2*shi2*(Iy-Iz) + Ip*omg_cap*tta2;
% 3
ttad1 = asin(Fxd);
ttad2 = dFxd/cos(tta1);
etta1 = ttad1 - tta1;
etta2 = ttad2 - tta2;
s3 = etta2 + lmbda(3)*etta1;
vr3 = ((Ip/Iy)*rho*abs(phi2) + K(3))*sat(s3,0.1);
u3 = Iy*(-1*phi2*shi2*((Iz-Ix)/Iy) + lmbda(3)*etta2 - (Ip/Iy)*omg_cap*phi2 + vr3);
% 4
eshi1 = 0 - shi1;
eshi2 = 0 - shi2;
s4 = eshi2 + lmbda(4)*eshi1;
u4 = Iz*(K(4)*sat(s4,0.1)-lmbda(4)*shi2);
u = [u1 u2 u3 u4]';

%Dynamics------------------

w = Alloc*u;
w1 = sqrt(w(1));
w2 = sqrt(w(2));
w3 = sqrt(w(3));
w4 = sqrt(w(4));

wa = [w1 w2 w3 w4];
omg = w1-w2+w3-w4;


z3 = (1/m)*(cos(phi1)*cos(tta1))*u1 - g;
phi3 = tta2*shi2*((Iy-Iz)/Ix) - (Ip/Ix)*omg*tta2 + (u2/Ix);
tta3 = phi2*shi2*((Iz-Ix)/Iy) + (Ip/Iy)*omg*phi2 + (u3/Iy);
shi3 = u4/Iz;

dxdt = [x2 y2 z2 phi2 tta2 shi2 x3 y3 z3 phi3 tta3 shi3]';

end