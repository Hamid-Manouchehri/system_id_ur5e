%%% identification procedure
% for velocity filter : [b,a] = butter(3,0.05,'low');
% for torque maybe decimate()

clc
clear all
close all

load regressor_reduced
load theta_reduced

syms clmb_1 visc_1 clmb_2 visc_2 clmb_3 visc_3 clmb_4 visc_4 clmb_5 visc_5 clmb_6 visc_6  real
theta_reduced = [theta_reduced ; [clmb_1 visc_1 clmb_2 visc_2 clmb_3 visc_3 clmb_4 visc_4 clmb_5 visc_5 clmb_6 visc_6]' ];

%%

load('/home/hamid/projects/system_id_ur5e/schunk_system_id/data/April_16_exciting_traj/measures_3.txt')
measures = measures_3;
offset = 3/0.005;
st = offset+1;
ed=length(measures)-st;
q_meas = measures(st:ed,1:6);
qd_meas = measures(st:ed,7:12);
taw_meas = measures(st:ed,13:18);
disp (['Size of measured data = ' num2str(length(q_meas))])

%% filter qd and qdd
% [b,a] = butter(3,0.05,'low');  % butter filter
% qd_fil = filter(b,a,qd_meas);

decimation = 6;
t = 0 : 0.005*decimation : (length(q_meas)-1)*0.005;
m = length(t);
q_reduced = q_meas(decimation/2:decimation:end,:);

disp (['Size of measured data = ' num2str(m)])
for i=1:6
    qd_fil(:,i) = decimate(qd_meas(:,i),decimation);
end
figure
plot(t,qd_fil);
figure
plot(qd_meas);

qdd_gen = zeros(m,6);
for k = 3:(m-2)
    qdd_gen(k,:) = ( -qd_fil(k+2,:) + 8*qd_fil(k+1,:) - 8*qd_fil(k-1,:) + qd_fil(k-2,:) ) / (3*( t(k+2) - t(k-2) ));
end
qdd_gen(1,:) = qdd_gen(3,:);
qdd_gen(2,:) = qdd_gen(3,:);
qdd_gen(end,:) = qdd_gen(end-2,:);
qdd_gen(end-1,:) = qdd_gen(end-2,:);
figure
plot(t,qdd_gen)

qdd_gen2 = zeros(m,6);
for i=1:6
    qdd_gen2(:,i) = smooth(qdd_gen(:,i));
end
figure
plot(t,qdd_gen2)

for i=1:6
    taw_fil(:,i) = decimate(taw_meas(:,i),decimation);
end
figure
plot(t,taw_fil);
figure
plot(taw_meas);
pause(1)

%% regressor
% hyperbolic = @(x,qdot) x(1).*tanh(x(2).*qdot)+x(3).*tanh(x(4).*qdot)+x(5).*qdot;
% load ERB145_fric.mat

step = 5;
vec = 1+step:step:m-step;
disp(['Number of samples for identification = ' num2str(length(vec))])
% Y_meas = zeros(4*length(vec),33);
Y_meas = zeros(6*length(vec),46+12);
% friction = zeros(4*length(vec),1);
cnt = 0;
for k = 1+step:step:m-step
    fprintf('point %d of %d\n',k,m);
    cnt = cnt+1;
    
    qs = [];
    qs = num2cell(q_reduced(k,:));
    [q1 q2 q3 q4 q5 q6] = deal(qs{:});
    
    qds = [];
    qds = num2cell(qd_fil(k,:));
    [dq1 dq2 dq3 dq4 dq5 dq6] = deal(qds{:});
    
    qdds = [];
    qdds = num2cell(qdd_gen2(k,:));
    [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6] = deal(qdds{:});
%     
%     q1 = q_reduced(k,1);
%     q2 = q_reduced(k,2);
%     q3 = q_reduced(k,3);
%     q4 = q_reduced(k,4);
%     q5 = q_reduced(k,5);
%     q6 = q_reduced(k,6);
%     dq1 = qd_fil(k,1);
%     dq2 = qd_fil(k,1);
%     dq3 = qd_fil(k,1);
%     dq4 = qd_fil(k,1);
%     dq5 = qd_fil(k,1);
%     dq6 = qd_fil(k,1);
%     ddq1 = traj(k,13);
%     ddq2 = traj(k,14);
%     ddq3 = traj(k,15);
%     ddq4 = traj(k,16);
%     ddq5 = traj(k,17);
%     ddq6 = traj(k,18);
    Y_num = eval(subs(regressor_red));
    for j=1:6
        Y_num(j,46+2*(j-1)+1 : 46+2*(j-1)+2) = [sign(qd_fil(k,j)) qd_fil(k,j)];
    end
    Y_meas(6*(cnt-1)+1: 6*cnt , 1:46+12) = Y_num;
    
%     Y_meas(4*(cnt-1)+1: 4*cnt , 1:33) = Y_num(1:4,:);
%     friction(4*(cnt-1)+1: 4*cnt , 1) = transpose(hyperbolic(ERB145,qd_fil(k,1:4)));
end
save Y_meas Y_meas;

%%
step = 5;
vec = 1+step:step:m-step;
load Y_meas.mat

%% identify parameters
taw_col = [];
for n=1:length(vec)
    taw_col = [taw_col ; taw_fil(vec(n),:)'];
end
% taw_pure = taw_col - friction;
taw_pure = taw_col;

theta_est = pinv(Y_meas)*taw_pure;
error = abs(Y_meas*theta_est - taw_pure);
figure;
plot(error(1:6:end)); hold on
plot(error(2:6:end),'r');
plot(error(3:6:end),'g');
plot(error(4:6:end),'k');
plot(error(5:6:end),'c');
plot(error(6:6:end),'y');

taw_est = Y_meas*theta_est;
for i=1:6
    figure()
    plot(t,taw_fil(:,i)); hold on
%     plot(taw_est(i:4:end)+friction(i:4:end),'r')
    plot(t(vec),taw_est(i:6:end),'r');
    plot(t(vec),error(i:6:end),'k');
end

for i=1:46+12
    disp([num2str(i) ' = ' char(theta_reduced(i)) ' = ' num2str(theta_est(i))])
end

%%

ub = 1*ones(33+12,1);
lb = -ub;
lb([5,6,8,9,12,13,15,16,19,20,22,23,25,26,27,28,29,31,32,33])= 0; %positives

ub([1,3,4,10,11,17,18,24]) = 4;   % first momentoms
lb([1,3,4,10,11,17,18,24]) = -4;

ub([32,33]) = 10;  % mass sumations

ub([34,36,38,40]) = 5.2;  % columbs erb145
lb([34,36,38,40]) = 4.2;

ub([35,37,39,41]) = 18;  % Viscous erb145
lb([35,37,39,41]) = 15;

ub([42,44]) = 0.33;  % columbs erb115
lb([42,44]) = 0.29;

ub([43,45]) = 0.52;  % Viscous erb115
lb([43,45]) = 0.4;

% x = lsqlin(C,d,A,b,Aeq,beq,lb,ub,x0,options)
[theta_est2,resnorm,residual,exitflag,output,lambda] = lsqlin(Y_meas,taw_pure,[],[],[],[],lb,ub,[],[]);
exitflag
for i=1:33+12
    disp([num2str(i) ' = ' char(theta_reduced(i)) ' = ' num2str(theta_est2(i))])
end
taw_est2 = Y_meas*theta_est2;
for i=1:6
    figure()
    plot(taw_fil(vec,i)); hold on
%     plot(taw_est(i:4:end)+friction(i:4:end),'r')
    plot(taw_est2(i:6:end),'r')
end
%% verification
joint = 3;
error = taw_col - Y_meas*theta_est;

figure
plot(error(5:6:end))






