
%%% Implementation of Feedback and Feedforward Controllers for velocity field%%%
clear all; clc; close all; set(0,'DefaultFigureWindowStyle','docked'); tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modelless learning of feedforward command as a function of time to
% compensate for a VF.
% 
% DESIRED TRAJECTORY IN CARTESIAN COORDINATES (column is time)
% NFTraj = [x_position; x_velocity; y_position; y_velocity];
%
% DESIRED TRAJECTORY IN JOINT COORDINATES (column is time)
% qDesired = [angle_shoulder; angle_velocity_shoulder;
%             angle_elbow;    angle_velocity_elbow];
%
%%%% DATA ORGANISATION %%%%
% All data is stored in a struct "Data1", which contains structs I-V that
% correspond to the conditions in the experiment. Each condition struct is
% populated by trials labelled T1-Tx.
%
% To get the data, use "eval" like in this example:
%
% for Trial=1:10
%   eval(strcat('Data.I_NF.T',num2str(Trial)));
% end
%
% ^ this enables you to dynamically assign or select data by using the same
% commands in MATLAB, but passed as a string statement.
%
%
% Atsushi Takagi 10/02/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load reference trajectory for controller and experimental data
load Tutorial4.mat;
% Name of experimental conditions in data
FieldName = char('I_NF','II_VF','III_Channel','IV_VF','V_NF');
SizeStruct = zeros(1,size(FieldName,1));
% Calculate block size of each condition
for k=1:size(FieldName,1)
    SizeStruct(1,k) = length(fieldnames(eval(strcat('Data1.(strtrim(FieldName(',num2str(k),',:)))'))));
end
% Parameters for the arm
Mass_S = 3; Mass_E = 3;                   % in kg
Length_S = 0.4; Length_E = 0.3;           % in m
CoM_S = Length_S/2; CoM_E = Length_E/2;   % in m
MoI_S = 0.125; MoI_E = 0.125;             % in kg.m^2
% PD parameters
P = 150; D = 60;
% Channel parameters
KChan = 10000;                            % in N/m
% Simulation step size
dt = 0.002;                               % in s
% Experiment trial structure
ExptTrialStruct = [0,SizeStruct(1),sum(SizeStruct(1:2)),sum(SizeStruct(1:3)),sum(SizeStruct(1:4)),sum(SizeStruct(1:5))]+1;
% Velocity-dependent force field in CARTESIAN COORDINATES
CurlField = [0 25; -25 0];                % in Ns/m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation structure (NF, VF, channel, VF, NF) for simulation
TrialStructure = ceil(ExptTrialStruct/5); % DO NOT CHANGE AS YOUR SIMULATIONS WILL LOOK DIFFERENT!

% FF parameters (alpha = proportion of PD learnt, gamma = forgetting)
Alpha = 0.08; Gamma = 0.001;

% Curl velocity force
CurlForce = @(x)((CurlField*x')');

% Euler method
UpdateSystem = @(x,a)([x(1,1)+dt*x(2,1); x(2,1)+a(1,1)*dt;
                       x(3,1)+dt*x(4,1); x(4,1)+a(2,1)*dt]);
                
% Forward kinematics            
ForwardKin = @(q)[Length_S*cos(q(1,:,:))+Length_E*cos(q(1,:,:)+q(2,:,:));
                  Length_S*sin(q(1,:,:))+Length_E*sin(q(1,:,:)+q(2,:,:))];
           
% Inverse kinematics
InvKin = @(x,y)[atan2(y,x)-acos((x.^2+y.^2+Length_S^2-Length_E^2)./(2*Length_S*sqrt(x.^2+y.^2)));
                acos((x.^2+y.^2-Length_S^2-Length_E^2)/(2*Length_S*Length_E))];

% Jacobian
Jacobian = @(q)[-Length_S*sin(q(1,:,:))-Length_E*sin(q(1,:,:)+q(2,:,:)), -Length_E*sin(q(1,:,:)+q(2,:,:));
                Length_S*cos(q(1,:,:))+Length_E*cos(q(1,:,:)+q(2,:,:)), Length_E*cos(q(1,:,:)+q(2,:,:))];

% Mass matrix
MassMatrix = @(q)[MoI_S+MoI_E+Mass_S*CoM_S^2+Mass_E*(Length_S^2+CoM_E^2+2*Length_S*CoM_E*cos(q(2,1))), MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2,1)));
                         MoI_E+Mass_E*(CoM_E^2+Length_S*CoM_E*cos(q(2,1))), MoI_E+Mass_E*CoM_E^2];

% Dynamics
JointAccel = @(Torque,JointAngle,JointVel,H)(H\(Torque-[Mass_E*Length_S*CoM_E*JointVel(2,1)*sin(JointAngle(2,1))*(2*JointVel(1,1)+JointVel(2,1));
                                                        Mass_E*Length_S*CoM_E*JointVel(1,1)^2*sin(JointAngle(2,1))]));

% Allocate memory
uPD = zeros(2,length(NFTraj),TrialStructure(end));
uFF = uPD;
uExt = uPD;
Force = uPD;

% Desired joint trajectories
qDesired = InvKin(NFTraj(1,:),NFTraj(3,:));
qDesired = [qDesired(1,:);
            0,diff(qDesired(1,:))/dt;
            qDesired(2,:);
            0,diff(qDesired(2,:))/dt];

q = zeros(4,size(qDesired,2),TrialStructure(end));
q(:,1,:) = repmat(qDesired(:,1),[1,1,TrialStructure(end)]);

% Experiment simulation loop
for Trial=1:TrialStructure(end)
 
end

toc




