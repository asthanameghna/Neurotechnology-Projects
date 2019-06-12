%%% Linear Quadratic Gaussian Filtering: Motion control with perfect sensory feedback %%%
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

R = 0.001;
Q = [0.5, 0;
     0, 0];

% Observation matrix
C = [1 0];

% target
yr = 0.05*ones(T,1);
tgtSize = 0.01;

% Generate  noise signals
Sigma_eta = 0.01;
Sigma_w = 10^-8;

eta = sqrt(Sigma_eta)*randn(size(Sigma_eta,1),T);
w = sqrt(Sigma_w)*randn(size(Sigma_w,1),T);

% initial conditon and the two targets
z = zeros(2,T);
z_b = zeros(2,T);
zinit = [0   0]; z(:,1) = zinit;
target = [0.2 0]';
newtarget = [-0.2 0]';
u = zeros(1,T);
u_b = zeros(1,T);

% define the time when the target change
TargetMove = 30;

fl = -10; fu = 10; %limits of force graph
yl = -0.3; yu = 0.3;% limits of position graph

for i = 1:T
    zr = target;
    yb(:,i) = C*z_b(:,i);
    L_b = dlqr(A,B,Q,R); % input your control law here
    u_b(:,i) = -L_b*(z_b(:,i)-zr);
    z_b(:,i+1) = A*z_b(:,i) + B*(u_b(:,i)+eta(i));
end

for i = 1:T
    if i>=TargetMove
        zr = newtarget;
    else
        zr = target;
    end
    y(:,i) = C*z(:,i);
    L = dlqr(A,B,Q,R); % input your control law here;
    u(:,i) = -L*(z(:,i)-zr);
    z(:,i+1) = A*z(:,i) + B*(u(:,i)+eta(i));
end

% plot the position
f1=figure(1);clf(1);set(gcf,'color','white'); set(gca,'fontsize',15);
hold all;

plot(t,y,'b-','linewidth',4);
plot(t,yb,'m-','linewidth',4);
h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);
line([min(t),max(t)], [target(1)-tgtSize, target(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [target(1)+tgtSize, target(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)-tgtSize, newtarget(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)+tgtSize, newtarget(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);

legend('Change target','No Target change');
xlabel('Time (s)','fontsize',20);
ylabel('Position (m)','fontsize',20);
ylim([yl yu]);

% plot the force profiles
f2 = figure(2);clf(2);set(gcf,'color','white'); set(gca,'fontsize',15);
hold all;

plot(t,u,'b-','linewidth',4);
plot(t,u_b,'m-','linewidth',4);

h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);

legend('Change target','No Target change');
xlabel('Time (s)','fontsize',20);
ylabel('force (N)','fontsize',20);
ylim([fl fu]);