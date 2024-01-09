clc
clear
close all
% solving the spring damper system
w = 2*pi;  % natural frequency
d = .25;   %damping ratio

%%spring  mass damper system
A = [0 1; -w^2 -2*d*w];  % \dot{x} = Ax
dt = 0.01;
T = 10;
x0 = [2;0];
%iterate the forward euler
xF(:,1) = x0;
tF(1) = 0;
for k = 1:T/dt
    tF(k+1) = k*dt;
    xF(:,k+1) = (eye(2)+dt*A)*xF(:,k);
end

plot(tF,xF(1,:),'k')

[t,xGood] = ode45(@(t,x) A*x,0:dt:T,x0);
hold on
plot(t,xGood(:,1),'r')
xlabel('Time[s]')
ylabel('Position [m]')
legend('Forward Euler','ODE45 (RK4)')

figure
plot(xGood(:,1),xGood(:,2),'k')
xlabel('Position [m]')
ylabel('Velocity [m/s]')
