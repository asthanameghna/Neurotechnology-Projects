clear all; clc; close all; set(0,'DefaultFigureWindowStyle','docked')

% time
dt = 0.001;
m = 0.005;
t = 0:dt:1;
T = size(t,2);
% System matrix
A = [1 dt;
    0  1];
B = [dt^2/2/m dt/m]';

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
zinit = [0   0];zb(:,1) = zinit;
target = [0.2 0]';
newtarget = [-0.2 0]';

% define the time when the target change
TargetMove = 70;



%%
%% Question 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%% construct your LQR controller here %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% name your controller gain L_b for no change of target and L for change of
% target
fl = -10; fu = 10; %limits of force graph
yl = -0.3; yu = 0.3;% limits of position graph
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L = dlqr(A,B,Q,R);

for i = 1:T
    zr = target;
    y(:,i) = C*z(:,i);
    % input your control law here;
    %
    z(:,i+1) = A*z(:,i) + B*(u(:,i)+eta(i));
end

for i = 1:T
    if i>=TargetMove
        zr = newtarget;
    else
        zr = target;
    end
    y(:,i) = C*z(:,i);
    % input your control law here;
    %
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
plot(t,fb,'m-','linewidth',4);

h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);

legend('Change target','No Target change');
xlabel('Time (s)','fontsize',20);
ylabel('force (N)','fontsize',20);
ylim([fl fu]);
% saveas(f1,'noise.eps','epsc');
pause
%%
%% for Question 2 and 3, use the sample code given below as a starting point for your own code
fl = -10; fu = 10;%limits of force graph
z(:,1) = zinit; z_L1 = z; z_L2 = z;
zhat_L1 = z(:,1); zhat_L2 = z(:,1);
u(:,1) = 0; u_L1 = u; u_L2 = u;
y(:,1) = C*z(:,1); y_L1 =y; y_L2 = y;
    
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
    y_L1(:,i) = C*z_L1(:,i)+w(:,i);
    y_L1_free(:,i) = C*z_L1(:,i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % place your controller here and 
    % fill out the forward model function

    zhat_L1(:,i+1) = ForwardModel();
    z_L1(:,i+1) = A*z_L1(:,i) + B*(u_L1(:,i)+eta(:,i));
end

disp('q2 - u_K1'); max(abs(u_L1))

%simulate LQR with Observer 2
for i = 1:T
    if i>=TargetMove
        zr = newtarget;
    else
        zr = target;
    end
    y_L1(:,i) = C*z_L1(:,i)+w(:,i);
    y_L1_free(:,i) = C*z_L1(:,i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % place your controller here and 
    % fill out the forward model function
    [zhat_L1(:,i+1), P] = KalmanFilter();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_L1(:,i+1) = A*z_L1(:,i) + B*(u_L1(:,i)+eta(:,i));
end

f3=figure(3);clf(3);set(gcf,'color','white'); set(gca,'fontsize',15);
hold on;

plot(t,y_L1_free,'b-','linewidth',8);
plot(t,y_L2_free,'k-','linewidth',4);


h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);
line([min(t),max(t)], [target(1)-tgtSize, target(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [target(1)+tgtSize, target(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)-tgtSize, newtarget(1)-tgtSize],'linewidth',2,'Color',[.8 .8 .8]);
line([min(t),max(t)], [newtarget(1)+tgtSize, newtarget(1)+tgtSize],'linewidth',2,'Color',[.8 .8 .8]);

legend('high L gain', 'low L gain');
xlabel('Time (s)','fontsize',20);
ylabel('Position (m)','fontsize',20);
ylim([yl yu]);

% plot the force profiles
f4 = figure(4);clf(4);set(gcf,'color','white'); set(gca,'fontsize',15);
hold all;

plot(t,u_L1,'b-','linewidth',4);
plot(t,u_L2,'k-','linewidth',4);

h = line(repmat(TargetMove*dt,2,1),[-zr(1,ones(1,size(TargetMove,2)));zr(1,ones(1,size(TargetMove,2)))],'linewidth',2);

legend('high L gain', 'low L gain');
xlabel('Time (s)','fontsize',20);
ylabel('force (N)','fontsize',20);
ylim([fl fu]);
