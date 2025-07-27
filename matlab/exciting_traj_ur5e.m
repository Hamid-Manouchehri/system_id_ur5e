%{
*******************************************************************************
Project: system-id ur5e
File: exciting_traj.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: June 22, 2025

Description:
Script for reading the calculated symbolic regressor and solving for the 
optimization problem to deriving the exciting trajectories.

License:
This script is licensed under the MIT License.
You may obtain a copy of the License at
    https://opensource.org/licenses/MIT

SPDX-License-Identifier: MIT

Disclaimer:
This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose, and noninfringement. In no event shall the
authors be liable for any claim, damages, or other liability, whether in an
action of contract, tort, or otherwise, arising from, out of, or in connection
with the software or the use or other dealings in the software.

*******************************************************************************
%}

clc; 
clear; 
close all;

% Load the symbolic regressor
tmp   = load('../data/mat/Y_sym.mat','Y_sym');
Y_sym = tmp.Y_sym;
Y_fun = @Y_fun;  % Y_fun(q', dq', ddq'); q, dq, and ddq are column vectors

% Saving optimized exciting trajectory
name_of_traj_file = "exciting_traj_test.csv";  % TODO
path_to_save_traj = "/home/hamid/projects/system_id_ur5e/data/traj/";
opt_traj_file = fullfile(path_to_save_traj, name_of_traj_file);

Wf = 2*pi/10;
N = 4;     % number of frequences
J = 6;     % number of joints
lambda1 = 1;
lambda2 = 1;

dt = 0.05;
T = 20;
t = linspace(0,T,1/dt);

x0 = ones(J*2*N,1);
A = [];

for i=1:length(t)
    A = [A ; cal_A(t(i),J,N,Wf)];
end

q_min   = [-360 -180 -90 -180 -90 -360]';  % in deg; [q1_min q2_min q3_min q4_min q5_min q6_min]'
q_max   = [ 360   0.  90  0    90  360]';  % in deg; 
dq_max  = [180 180 180 180 180 180]';
ddq_max = [180 180 180 180 180 180]';

Q_max = [q_max; dq_max; ddq_max]*pi/180;  % in rad
Q_min = [q_min; dq_max; ddq_max]*pi/180;  % in rad
Q_total = repmat([Q_max;-Q_min],length(t),1);
objFun = @(x) regressor_cond(x, A, t, lambda1, lambda2, Y_fun);
% Qdd_max for UR5e: 13 rad/s^2 

Ain = [A;-A]; bin = Q_total;
Aeq = [];     beq = [];
lb  = [];     ub  = [];
nonlcon = [];
opts = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'FiniteDifferenceType','forward', ...
    'UseParallel',true, ...
    'MaxFunctionEvaluations',3e2, ...
    'PlotFcn',{ @optimplotfval, ...           % objective vs iter
                @optimplotfirstorderopt, ...  % KKT optimality
                @optimplotstepsize, ...       % step size
                @optimplotconstrviolation }); % max constraint violation);

[x,fval,exitflag] = fmincon(objFun,x0,Ain,bin,Aeq,beq,lb,ub,nonlcon,opts);

A = [];

for i = 1:length(t)
    A = [A ; cal_A(t(i),J,N,Wf)];
end

Q = A*x;

disp("saving optimal trajectory...");

Ns = length(t);
q1_opt = Q(1:18:end);
q2_opt = Q(2:18:end);
q3_opt = Q(3:18:end);
q4_opt = Q(4:18:end);
q5_opt = Q(5:18:end);
q6_opt = Q(6:18:end);
Q_opt = [q1_opt, q2_opt, q3_opt, q4_opt, q5_opt, q6_opt];

traj_data = [t(:), Q_opt];
fid = fopen(opt_traj_file,'w');
fprintf(fid,'time,q1,q2,q3,q4,q5,q6\n');  % add header
fclose(fid);
writematrix(traj_data, opt_traj_file, 'WriteMode','append');


% plotting exciting trajectory
figure;
plot(t,q1_opt,'b'); hold on;
plot(t,q2_opt,'r');
plot(t,q3_opt,'g');
plot(t,q4_opt,'k');
plot(t,q5_opt,'c');
plot(t,q6_opt,'m');

%%% Construct A matrix (can be used for trajectory generation 

%%% and optimization linear condition as well . Ax = Q

function A = cal_A(t,J,N,Wf)

    A = zeros(3*J , 2*N*J);
    sub_q = [];
    sub_qd = [];
    sub_qdd = [];

    for l=1:N

        sub_q = [sub_q , 1/(Wf*l)*[sin(Wf*l*t) , -cos(Wf*l*t)] ];
        sub_qd = [sub_qd , [cos(Wf*l*t) , sin(Wf*l*t)] ];
        sub_qdd = [sub_qdd , (Wf*l)*[-sin(Wf*l*t) , cos(Wf*l*t)] ];

    end

    sub_Q = [sub_q ; sub_qd ; sub_qdd];

    for n=1:3
        for i=1:J
            A( (n-1)*J+i , 2*N*(i-1)+1:2*N*i ) = sub_Q(n,:);
        end
    end
end

function R = regressor_cond(X, A, t, lambda1, lambda2, Y_fun)

    Q = A*X;
    R = 0;

    % q1_traj = zeros(length(t),1);
    % for i=1:length(t)
    %     q1_traj(i) = Q((i-1)*18+1);
    % end
    % plot(t, q1_traj);

    for k=1:length(t)
        [q1, q2, q3, q4, q5, q6, ...
        dq1, dq2, dq3, dq4, dq5, dq6, ...
        ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ...
        R_10xx, R_10xy, R_10xz, R_10yx, R_10yy, R_10yz, R_10zx, R_10zy, R_10zz, ... 
        R_21xx, R_21xy, R_21xz, R_21yx, R_21yy, R_21yz, R_21zx, R_21zy, R_21zz, ...
        R_32xx, R_32xy, R_32xz, R_32yx, R_32yy, R_32yz, R_32zx, R_32zy, R_32zz, ...
        R_43xx, R_43xy, R_43xz, R_43yx, R_43yy, R_43yz, R_43zx, R_43zy, R_43zz, ...
        R_54xx, R_54xy, R_54xz, R_54yx, R_54yy, R_54yz, R_54zx, R_54zy, R_54zz, ...
        R_65xx, R_65xy, R_65xz, R_65yx, R_65yy, R_65yz, R_65zx, R_65zy, R_65zz, ...
        ddp1_x, ddp1_y, ddp1_z, ddp2_x, ddp2_y, ddp2_z, ddp3_x, ddp3_y, ddp3_z, ...
        ddp4_x, ddp4_y, ddp4_z, ddp5_x, ddp5_y, ddp5_z, ddp6_x, ddp6_y, ddp6_z, ...
        w1_x, w1_y, w1_z, w2_x, w2_y, w2_z, w3_x, w3_y, w3_z, ...
        w4_x, w4_y, w4_z, w5_x, w5_y, w5_z, w6_x, w6_y, w6_z, ...
        dw1_x, dw1_y, dw1_z, dw2_x, dw2_y, dw2_z, dw3_x, dw3_y, dw3_z, ...
        dw4_x, dw4_y, dw4_z, dw5_x, dw5_y, dw5_z, dw6_x, dw6_y, dw6_z, ...
        r01_x, r01_y, r01_z, r12_x, r12_y, r12_z, r23_x, r23_y, r23_z, ...
        r34_x, r34_y, r34_z, r45_x, r45_y, r45_z, r56_x, r56_y, r56_z] = Kinematic_Param_ur5e(Q( (k-1)*18+1 : (k-1)*18+18 ));

        Y_num = Y_fun([q1,q2,q3,q4,q5,q6]',...
                      [dq1,dq2,dq3,dq4,dq5,dq6]',...
                      [ddq1,ddq2,ddq3,ddq4,ddq5,ddq6]');
        s = svd(Y_num);
        R = R + ( lambda1*cond(Y_num) + lambda2/( s(nnz(s))) );

    end
    % disp(['R = ' num2str(R)])
end

