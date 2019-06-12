%%% LQG and Kalman Filtering %%%

clear; clc;

% time
dt = 0.001;
m = 0.005;
t = 0:dt:1;
T = size(t,2);
% System matrix
A = [1 dt;
    0  1];
B = [dt^2/2/m dt/m]';

R = 0.005;
Q = [1, 0;
     0, 0];

%Forward Model Gain
K1 = [0.5; 145];
K2 = [0.004; 0.005];

% Observation matrix
C = [1 0];

% target
yr = 0.05*ones(T,1);
tgtSize = 0.01;

% Generate  noise signals
Sigma_eta = 10^-4;
Sigma_w = 0.01;

eta = sqrt(Sigma_eta)*randn(size(Sigma_eta,1),T);
w = sqrt(Sigma_w)*randn(size(Sigma_w,1),T);

% initial conditon and the two targets
zinit = [0   0];zb(:,1) = zinit;
target = [0.2 0]';
newtarget = [-0.2 0]';
u_L1 = zeros(1, T);
u_L2 = zeros(1, T);
u_K = zeros(1, T);
zhat_L1 = zeros(2, T);
zhat_L2 = zeros(2, T);
zhat_K = zeros(2, T);
y_L1 = zeros(1, T);
y_L2 = zeros(1, T);
y_K = zeros(1, T);
z_L1 = zeros(2, T);
z_L2 = zeros(2, T);
z_K = zeros(2, T);

% define the time when the target change
TargetMove = 30;

fl = -10; fu = 10;%limits of force graph
yl = -0.3; yu = 0.3;% limits of position graph
    
% Initialize error covariance matrices
PInit =1e-3*diag([1 1]);
P = PInit;

%simulate LQR with Observer 1
for i = 1:T
    if i>=TargetMove
        zr = newtarget;
    else
        zr = target;
    end
    
     y_L1_free(1,i) = C*z_L1(:,i)+w(:,i);
      y_L2_free(1,i) = C*z_L2(:,i)+w(:,i);
    
    L1 = dlqr(A,B,Q,R); % input your control law here
    L2 = L1;
    u_L1(:,i) = -L1*(zhat_L1(:,i)-zr);
    u_L2(:,i) = -L2*(zhat_L2(:,i)-zr);

    zhat_L1(:,i+1) = ForwardModel(A,B,C,K1,z_L1(:,i),y_L1_free(1,i),u_L1(1,i));
    z_L1(:,i+1) = A*z_L1(:,i) + B*(u_L1(1,i)+eta(1,i));
    
    zhat_L2(:,i+1) = ForwardModel(A,B,C,K2,z_L2(:,i),y_L2_free(1,i),u_L2(1,i));
    z_L2(:,i+1) = A*z_L2(:,i) + B*(u_L2(1,i)+eta(1,i));
end

disp('q2 - u_K1'); max(abs(u_L1))

%simulate LQR with Observer 2
for i = 1:T
    if i>=TargetMove
        zr = newtarget;
    else
        zr = target;
    end

    y_K_free(:,i) = C*z_K(:,i)+w(:,i);
    LK = dlqr(A,B,Q,R); % input your control law here
    u_K(:,i) = -LK*(zhat_K(:,i)-zr);

    [zhat_K(:,i+1) P] = KalmanFilter(A,B,C,Sigma_eta,Sigma_w,zhat_K(:,i),P,y_K_free(1,i),u_K(1,i));
    z_K(:,i+1) = A*z_K(:,i) + B*(u_K(1,i)+eta(1,i));
end

f3=figure(3);clf(3);set(gcf,'color','white'); set(gca,'fontsize',15);
hold on;

plot(t,y_L1_free,'b-','linewidth',4);
plot(t,y_L2_free,'k-','linewidth',4);
plot(t,y_K_free,'g-','linewidth',1);
h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);
line([min(t),max(t)], [target(1)-tgtSize, target(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [target(1)+tgtSize, target(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)-tgtSize, newtarget(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)+tgtSize, newtarget(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
legend('LQG Filtering K1', 'LQG Filtering K2', 'Kalman Filtering');
xlabel('Time (s)','fontsize',20);
ylabel('Position (m)','fontsize',20);

% % plot the force profiles
f4 = figure(4);clf(4);set(gcf,'color','white'); set(gca,'fontsize',15);
hold all;
plot(t,u_L1,'b-','linewidth',3);
plot(t,u_L2,'k-','linewidth',6);
plot(t,u_K,'g-','linewidth',2);
h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);
legend('LQG Filtering K1', 'LQG Filtering K2', 'Kalman Filtering');
xlabel('Time (s)','fontsize',20);
ylabel('force (N)','fontsize',20);
ylim([fl fu]);
