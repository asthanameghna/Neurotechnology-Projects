/* Simulation of Motor Adaptation in a Novel Force Field */

clear
clc
load Tutorial4.mat;
FieldName = char('I_NF','II_VF','III_Channel','IV_VF','V_NF');
SizeStruct = zeros(1,size(FieldName,1));

for k=1:size(FieldName,1)
    SizeStruct(1,k) = length(fieldnames(eval(strcat('Data1.(strtrim(FieldName(',num2str(k),',:)))'))));
end

TrialNo = sum(SizeStruct);
Trials = zeros(2,TrialNo);
Tcounter = 1;

%Null Field
for TrialsNF =1:SizeStruct(1,1)
  X_NF = eval(strcat('Data1.I_NF.T',num2str(TrialsNF),'(:,1)'));
  Trials(2,Tcounter) = abs(max(X_NF));
  Trials(1,Tcounter) = Tcounter;
  Tcounter = Tcounter+1;
end

%Velocity Dependent Field
for TrialsVF =1:SizeStruct(1,2)
  X_VF = eval(strcat('Data1.II_VF.T',num2str(TrialsVF),'(:,1)'));
  Trials(2,Tcounter) = abs(max(X_VF));
  Trials(1,Tcounter) = Tcounter;
  Tcounter = Tcounter+1;
end

%Force Channel
for TrialsFC =1:SizeStruct(1,3)
  X_FC = eval(strcat('Data1.III_Channel.T',num2str(TrialsFC),'(:,1)'));
  Trials(2,Tcounter) = abs(max(X_FC));
  Trials(1,Tcounter) = Tcounter;
  Tcounter = Tcounter+1;
end

%Re Testing VF
for Trials4VF =1:SizeStruct(1,4)
  X_4VF = eval(strcat('Data1.IV_VF.T',num2str(Trials4VF),'(:,1)'));
  Trials(2,Tcounter) = abs(max(X_4VF));
  Trials(1,Tcounter) = Tcounter;
  Tcounter = Tcounter+1;
end

%Washout NF
for TrialsWNF =1:SizeStruct(1,5)
  X_WNF = eval(strcat('Data1.V_NF.T',num2str(TrialsWNF),'(:,1)'));
  Trials(2,Tcounter) = abs(max(X_WNF));
  Trials(1,Tcounter) = Tcounter;
  Tcounter = Tcounter+1;
end

    
subplot(2,5,6:10)
hold on
plot(Trials(1,:),Trials(2,:));
axis([0 348 0 0.08]);

%Fill Plot
X1 = [Trials(1,1:32),fliplr(Trials(1,1:32))];
Y1 = [ones(1,32)*0.08, fliplr(Trials(2,1:32))];
X2 = [Trials(1,32:159),fliplr(Trials(1,32:159))];
Y2 = [ones(1,128)*0.08, fliplr(Trials(2,32:159))];
X3 = [Trials(1,159:309),fliplr(Trials(1,159:309))];
Y3 = [ones(1,151)*0.08, fliplr(Trials(2,159:309))];
X4 = [Trials(1,309:321),fliplr(Trials(1,309:321))];
Y4 = [ones(1,13)*0.08, fliplr(Trials(2,309:321))];
X5 = [Trials(1,321:347),fliplr(Trials(1,321:347))];
Y5 = [ones(1,27)*0.08, fliplr(Trials(2,321:347))];
LDX = [Trials(1,:),fliplr(Trials(1,:))];
LDY = [Trials(2,:), fliplr(zeros(1,347))];
fill(X1,Y1,'r',X2,Y2,'g',X3,Y3,'y',X4,Y4,'b',X5,Y5,'m',LDX,LDY,'k');
legend('Max Absolute Lateral Deviation','Null Field','Velocity Force Field','Force Channel','Re-Test Velocity Force Field','Washout Null Field');
xlabel('Trials');ylabel('Maximum Absolute Lateral Deviation');


