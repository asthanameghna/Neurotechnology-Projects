/* Implementation of Feedback and Feedforward Controllers */

clear; clc;
load Tutorial4.mat;

% Parameters for the arm
Mass_S = 3; Mass_E = 3;                   % in kg
Length_S = 0.4; Length_E = 0.3;           % in m
CoM_S = Length_S/2; CoM_E = Length_E/2;   % in m
MoI_S = 0.125; MoI_E = 0.125;             % in kg.m^2

% Inverse kinematics
InvKin = [atan2(NFTraj(3,:),NFTraj(1,:))-acos((NFTraj(1,:).^2+NFTraj(3,:).^2+Length_S^2-Length_E^2)./(2*Length_S*sqrt(NFTraj(1,:).^2+NFTraj(3,:).^2)));
                acos((NFTraj(1,:).^2+NFTraj(3,:).^2-Length_S^2-Length_E^2)/(2*Length_S*Length_E))];

% Desired joint trajectories
qDesired = [InvKin(1,:);InvKin(2,:)];

qdotDesired = zeros(2,550);
dt = 0.002;
for t=2:550
qdotDesired(:,t) = [(qDesired(1,t)-qDesired(1,t-1))/dt; (qDesired(2,t)-qDesired(2,t-1))/dt];
end

CurlField = [0 25; -25 0];
q = zeros(2,550);
q(1,1)=0; q(2,1)=0.3101;
qdot = zeros(2,550);
qdotdot = zeros(2,550);
FinalTraj = zeros(2,550);
FinalTrajVel = zeros(2,550);

%%%% Velocity Dependent Force Field
% Initializing Variables
uFB = zeros(2,550);
uFF = zeros(2,550);
e = zeros(2,550);
edot = zeros(2,550);
u = zeros(2,550);
CurlForce = zeros(2,550);
Torque = zeros(2,550);

for period=1:550
%e values
e(:,period) = qDesired(:,period) - q(:,period);
edot(:,period) = qdotDesired(:,period) - qdot(:,period);

%Feedback Component
P = 150; D = 60;
uFB(:,period) = P*e(:,period) + D*edot(:,period);

%Feedforward Component
Alpha = 0.08; Gamma = 0.001;
uFF(:,period+1) = Alpha*uFB(:,period) - (1-Gamma)*uFF(:,period);
   
%Torque Component

u(:,period) = uFB(:,period) + uFF(:,period);
Torque(:,period) = u(:,period) + CurlForce(:,period);

%Multi Joint Dynamics
H = [MoI_S+MoI_E+Mass_S*CoM_S^2+Mass_E*(Length_S^2+CoM_E^2+2*Length_S*CoM_E*cos(q(2,period))), MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2,period)));
                         MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2,period))), MoI_E+Mass_E*CoM_E^2];                 
                     
Q = [Mass_E*Length_S*CoM_E*qdot(2,period)*sin(q(2,period))*(2*qdot(1,period)+qdot(2,period));
                                                        Mass_E*Length_S*CoM_E*qdot(1,period)^2*sin(q(2,period))];
qdotdot(:,period) = (H\(Torque(:,period)-Q));
            
%Euler Update
dt = 0.002;
qdot(:,period+1) = qdot(:,period)+ qdotdot(:,period)*dt;
q(:,period+1)= q(:,period)+qdot(:,period)*dt;

% Forward kinematics            
FinalTraj(:,period) = [Length_S*cos(q(1,period))+Length_E*cos(q(1,period)+q(2,period));
                  Length_S*sin(q(1,period))+Length_E*sin(q(1,period)+q(2,period))];

% FinalTrajVel(:,period) = [(FinalTraj(1,period+1)-FinalTraj(1,period))/dt; (FinalTraj(2,period+1)-FinalTraj(2,period))/dt];
Jacobian = [-Length_S*sin(q(1,period))-Length_E*sin(q(1,period)+q(2,period)), -Length_E*sin(q(1,period)+q(2,period));
                Length_S*cos(q(1,period))+Length_E*cos(q(1,period)+q(2,period)), Length_E*cos(q(1,period)+q(2,period))];
FinalTrajVel(:,period) = Jacobian*qdot(:,period);      
CurlForce(:,period) = CurlField*FinalTrajVel(:,period);
end

plot(FinalTraj(1,:),FinalTraj(2,:));
hold on;
plot(NFTraj(1,:),NFTraj(3,:));
axis([-0.2 1 -0.1 0.7]);