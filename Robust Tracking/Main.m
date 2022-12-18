%Trajectory Generation
syms t real;
[xd,xd_dot,xd_ddot,t1] = Quintic_syms([0 0 1 1 0 0],[0 5 20 35 50 65],0.1);
[yd,yd_dot,yd_ddot,~] = Quintic_syms([0 0 0 1 1 0],[0 5 20 35 50 65],0.1);
[zd,zd_dot,zd_ddot,~] = Quintic_syms([0 1 1 1 1 1],[0 5 20 35 50 65],0.1);
shid = zeros([length(t1),1]);

% %--------
% figure(1);
% subplot(1,3,1)
% fplot(t,xd,[0 65]);
% subplot(1,3,2)
% fplot(t,xd_dot,[0 65]);
% subplot(1,3,3)
% fplot(t,xd_ddot,[0 65]);
% %--------
% figure(2);
% subplot(1,3,1)
% fplot(t,yd,[0 65]);
% subplot(1,3,2)
% fplot(t,yd_dot,[0 65]);
% subplot(1,3,3)
% fplot(t,yd_ddot,[0 65]);
% %-------
% figure(3);
% subplot(1,3,1)
% fplot(t,zd,[0 65]);
% subplot(1,3,2)
% fplot(t,zd_dot,[0 65]);
% subplot(1,3,3)
% fplot(t,zd_ddot,[0 65]);

K = double([10,400,450,8]);
lmbda = double([4,6,6,16]);
timespan = [0 5];
xtraj = [xd xd_dot xd_ddot]';
ytraj = [yd yd_dot yd_ddot]';
x0 = zeros(12,1);

[t2,yy2] = ode45(@(t1,x) crazy_controller(t1,x,K,lmbda),[0 65],x0);

%input calculation
len = length(t2);
w = zeros([4,len]);
a = zeros([12,len]);
% ue = zeros([2,len]);
% rho = zeros([1,len]);
for i=1:len
    [a(:,i),~,w(:,i)]=crazy_controller(t2(i),yy2(i,:),K,lmbda);
end


figure(1);

subplot(2,3,1);
hold on;
plot(t2(:,1),yy2(:,1));
subplot(2,3,2);
hold on;
plot(t2(:,1),yy2(:,2));
subplot(2,3,3);
hold on;
plot(t2(:,1),yy2(:,3));
hold on;
subplot(2,3,4);
plot(t2(:,1),rad2deg(yy2(:,4)));
hold on;
subplot(2,3,5);
plot(t2(:,1),rad2deg(yy2(:,5)));
hold on;
subplot(2,3,6);
plot(t2(:,1),yy2(:,6));


figure(2);
subplot(2,2,1);
hold on
plot(t2,w(1,:))
xlabel('t (sec)')
ylabel('w 1 (rad/sec)')
title('Rotor 1');
hold off
subplot(2,2,2);
hold on
plot(t2,w(2,:))
xlabel('t (sec)')
ylabel('w 2 (rad/sec)')
title('Rotor 2');
hold off
subplot(2,2,3);
hold on
plot(t2,w(3,:))
xlabel('t (sec)')
ylabel('w 3 (rad/sec)')
title('Rotor 3');
hold off
subplot(2,2,4);
hold on
plot(t2,w(4,:))
xlabel('t (sec)')
ylabel('w 4 (rad/sec)')
title('Rotor 4');
hold off

