/*In this tutorial you will examine the kinematics of a three joint arm composed of the wrist, forearm and
upper arm.
Used the minimization of speed criterion for a point-to-point
reaching movement by following a trajectory
Simulate this example in MATLAB and present graphs of the results (endpoint position,
endpoint velocity, joint angles and joint velocities as functions of time).*/

J = [0.0257 1.0257 0.383; -0.4446 -0.4446 0.3213; 1 1 1];
JT = transpose (J);
JJT = J*JT;
JJTInv = inv (JJT);
pseudoJ = JT*JJTInv;
pt=linspace (0,2,200);
x = zeros (1,200);
y = zeros (1,200);
theta = zeros (1,200);

xdt = zeros (1,200);
ydt = zeros (1,200);
thetadt = zeros (1,200);
qdtmatrix = zeros (3,200);

q1 = zeros (1,200);
q2 = zeros (1,200);
q3 = zeros (1,200);

for t = 1:200
  xdt(t) = 0.401 * ((30*(pt(t)^4)/(2^5))-(60*(pt(t)^3)/(2^4))+(30*(pt(t)^3)/(2^4)));
  ydt(t) = 2.456 * ((30*(pt(t)^4)/(2^5))-(60*(pt(t)^3)/(2^4))+(30*(pt(t)^3)/(2^4))); 
  x(t) = 0.401 * ((6*(pt(t)^5)/(2^5))-(15*(pt(t)^4)/(2^4))+(10*(pt(t)^4)/(2^4)))-0.445;
  y(t) = 2.456 * ((6*(pt(t)^5)/(2^5))-(15*(pt(t)^4)/(2^4))+(10*(pt(t)^4)/(2^4)))-0.026;
  if t == 1
  slope = y(t)/x(t);
  theta(1) = atan(slope);
  thetadt(1) = theta(1)-0/0.01;
  X = [xdt(t); ydt(t); thetadt(t)];
  qdt = pseudoJ*X;
  q1(1) = 90+(qdt(1)*0.01);
  q2(1) = 130+(qdt(2)*0.01);
  q3(1) = 90+(qdt(3)*0.01);
  
  else
      slope2 = y(t)/x(t);
      theta(t) = atan(slope2); 
      thetadt(t) = (theta(t)-theta(t-1))/0.01;
      X2 = [xdt(t); ydt(t); thetadt(t)];
      qdt2 = pseudoJ*X2;
      for i = 1:3
          qdtmatrix(i,t) = qdt2(i);
      end
      q1(t) = q1(t-1)+(qdt2(1)*0.01);
      q2(t) = q2(t-1)+(qdt2(2)*0.01);
      q3(t) = q3(t-1)+(qdt2(3)*0.01);
  end
end

plot (pt,q1)
xlabel('time in seconds');ylabel('qs');
figure
plot (pt,q2,'r')
xlabel('time in seconds');ylabel('qe');
figure
plot (pt,q3,'y')
xlabel('time in seconds');ylabel('qw');
figure
plot (pt,qdtmatrix(1,:))
xlabel('time in seconds');ylabel('shoulder joint velocity');
figure
plot (pt,qdtmatrix(2,:),'r')
xlabel('time in seconds');ylabel('elbow joint velocity');
figure
plot (pt,qdtmatrix(3,:),'y')
xlabel('time in seconds');ylabel('wrist joint velocity');
figure
plot3 (x, y, pt)
xlabel('x position');ylabel('y position');zlabel('time'); grid on;
figure
plot3 (xdt, ydt, pt)
xlabel('x velocity');ylabel('y velocity');zlabel('time'); grid on;