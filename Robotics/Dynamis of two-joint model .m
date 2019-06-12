/* Dynamis of two-joint model */
i = linspace (0,20,10000);

q_dot = zeros (2,10000);
q = zeros (2,10000);
q_deg = zeros (2,10000);
q_dd = zeros (2,10000);
Tau = zeros (2,10000);
C = zeros (2,10000);

H = [0.1146 0.0373; 0.0373 0.02];
dt = 0.002;
q(:,1) = [0.5235; 0];
q_deg(:,1) = [30; 0];
q_dot(:,1) = q(:,1)*dt;
C(:,1) = 0.02*sin(q(1,1))*[ q_dot(1,1)*(2*q_dot(2,1)+q_dot(1,1)); (q_dot(2,1))^2 ]; 

 
for j=1:9999
    
    if i(j)<2
        Tau(1,j) = 0.01 - 0.01*q_dot(1,j);
    else
        Tau(1,j) = -0.01*q_dot(1,j);
    end
    
    Tau(2,j) = -0.01*q_dot(2,j);
    C(:,j) = 0.02*sin(q(1,j))*[ q_dot(1,j)*(2*q_dot(2,j)+q_dot(1,j)); (q_dot(2,j))^2 ]; 
    q_dd(:,j) = inv(H)*(Tau(:,j)-C(:,j));
    q_dot(:,j+1) = q_dot(:,j) + q_dd(:,j)*dt;
    q(:,j+1) = q(:,j) + q_dot(:,j)*dt + 0.5*q_dd(:,j)*(dt^2);
    q_deg(:,j+1) = q(:,j+1)*180/3.142; 
     
end
 
plot (i,q_deg);
xlabel('Time (sec)');
ylabel('Elbow/Shoulder Angle (deg)');
title('Arm Trajectory');
legend('Elbow Angle','Shoulder Angle');
