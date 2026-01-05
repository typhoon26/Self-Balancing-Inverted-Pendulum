syms u thet(t) theta(t) theta_t(t) omega_0 e
gValue = 9.81;
rValue = 1;
%drag coefficient
c=.3;
%pid coefficients
p=1*0;
i=1*0;
d=.2*0;
%input coefficient
b=5;
%measures of error
e=theta-pi;
e_t=theta_t;
e_i=int(e,t);
%pid controller
u=-p*e-d*e_t-i*e_i;
omega_0Value = sqrt(gValue/rValue);
%state space equation
eqs = [diff(theta)   == theta_t;
       diff(theta_t) == -omega_0^2*sin(theta)-c*theta_t+b*u];
%substituting omega_0
eqs  = subs(eqs,omega_0,omega_0Value);
vars=[theta, theta_t];
[M,F] = massMatrixForm(eqs,vars);
f = M\F;
%defining ode function
f = odeFunction(f, vars);
%initial state angular position and angular velocity
x0 = [pi/2-.001; -2.0*omega_0Value];
%duration of Animation
tInit  = 0;
tFinal = 20;
%copying solution of ODE
sols = ode45(f,[tInit tFinal],x0);
%x and y positions of bob
x_pos = @(t) sin(deval(sols,t,1));
y_pos = @(t) -cos(deval(sols,t,1));
figure;
fanimator(@(t) plot(x_pos(t),y_pos(t),'ko','MarkerFaceColor','k'),'AnimationRange',[0 20]);
%hold on;
%fanimator(@(t) plot([0 x_pos(t)],[0 y_pos(t)],'k-'));
%fanimator(@(t) text(-0.3,1.5,"Timer: "+num2str(t,2)+" s"));
hold on;
%rod animation
fanimator(@(t) plot([0 x_pos(t)],[0 y_pos(t)],'k-'),'AnimationRange',[0 20]);
%text animation
fanimator(@(t) text(-0.3,0.3,"Timer: "+num2str(t,2)+" s"),'AnimationRange',[0 20]);
playAnimation;
