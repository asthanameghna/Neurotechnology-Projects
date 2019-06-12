/* Simulation of Dynamics of one-joint model*/
i = linspace (0,20,10000);

qe_dot = zeros (1,10000);
qe = zeros (1,10000);
qe_deg = zeros (1,10000);
qe_dd = zeros (1,10000);
TauE = zeros (1,10000);
C = zeros (1,10000);

H = 0.02;
dt = 0.002;
qe(1) = 0.5235;
qe_deg(1) = 30;
qe_dot(1) = qe(1)*dt;

 
for j=1:9999
    
    if i(j)<2
        TauE(j) = 0.01 - 0.01*qe_dot(j);
    else
        TauE(j) = -0.01*qe_dot(j);
    end
    
    C(j) = 0.02*sin(qe(j))* qe_dot(j)^2;
    qe_dd(j) = (TauE(j)-C(j))/H;
    qe_dot(j+1) = qe_dot(j) + qe_dd(j)*dt;
    qe(j+1) = qe(j) + qe_dot(j)*dt + 0.5*qe_dd(j)*(dt^2);
    qe_deg(j+1) = qe(j+1)*180/3.142; 
     
end
 
plot (i,qe_deg);
xlabel('Time (sec)');
ylabel('Elbow Angle (deg)');
title('Elbow Trajectory with Fixed Shoulder');