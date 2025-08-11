%{
*******************************************************************************
Project: system-id ur5e
File: id_validation.m
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Aug 10, 2025

Description:
Script for cross-checking the symbolic calculation of kinematics and 
dynamics of UR5e robot with robotic_system_toolbox.m.

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
clc; clear; close all;

tic

% Loading:
trajFile = 'ur5e_smooth_random_joint_traj_v1.csv';  % TODO
trajDir  = '../data/traj';
csvTraj = fullfile(trajDir, trajFile);
[hdr, tTraj, qTraj] = readCSVFile(csvTraj);

% Saving:
workspaceFile = 'ur5e_smooth_random_joint_traj_v1_analytical_workspace_test.mat';  % TODO
workspaceDir  = '../data/mat';
analyticalWorkspace = fullfile(workspaceDir, workspaceFile);

syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

qs = [q1 q2 q3 q4 q5 q6];

m1 = 3.761;
m2 = 8.058;
m3 = 2.846;
m4 = 1.37;
m5 = 1.3;
m6 = 0.365;

I_m1 = 6.6119e-5;
I_m2 = 4.6147e-5;
I_m3 = 7.9773e-5;
I_m4 = 1.2249e-5;
I_m5 = 1.1868e-5;
I_m6 = 1.1981e-5;

m1_rC1 = [0, -0.02561, 0.00193]';
m2_rC2 = [0.2125, 0, 0.11336]';
m3_rC3 = [0.15, 0.0, 0.0265]';
m4_rC4 = [0, -0.0018, 0.01634]';
m5_rC5 = [0, 0.0018,0.01634]';
m6_rC6 = [0, 0, -0.001159]';

I_1_h = [0.0067 -0.     -0.;
        -0.      0.0064 -0.;
        -0.     -0.      0.0067];

I_2_h = [0.0149 -0.     -0.;
        -0.      0.3564 -0.;
        -0.     -0.      0.3553];

I_3_h = [0.0025 -0.     -0.0034;
        -0.      0.0551 -0.;
        -0.0034 -0.      0.0546];

I_4_h = [0.0012 -0.     -0.;
        -0.      0.0012 -0.;
        -0.     -0.      0.0009];

I_5_h = [0.0012 -0.     -0.;
        -0.      0.0012 -0.;
        -0.     -0.      0.009];

I_6_h = [0.001 -0.    -0.;
        -0.     0.001 -0.;
        -0.    -0.     0.001];

% UR5e DH parameters:
TCP_offset_z = 0.171; % m
a     = [0.0 -0.425 -0.3922 0.0 0.0 0.0];
alpha = [pi/2 0.0 0.0 pi/2 -pi/2 0.0];
d     = [0.1625 0.0 0.0 0.1333 0.0977 0.0966 + TCP_offset_z];

% Gear ratios:
g = 9.81;
k_r1 = 100;
k_r2 = 100;
k_r3 = 100;
k_r4 = 100;
k_r5 = 100;
k_r6 = 100;

% Clculating symbolic transformation matrices for all links:
T_matrix = struct('T', cell(1,6));
% T_tcp_to_base = eye(4);

for i = 1:6
    % homogeneous transform of frame i relative to frame i-1
    T_matrix(i).T = dhTransform(a(i), alpha(i), d(i), qs(i));
    % T_tcp_to_base = T_tcp_to_base * T_matrix(i).T;
end

T_1 = T_matrix(1).T;  % Base
T_2 = vpa(T_1 * T_matrix(2).T, 2); % Shoulder w.r.t. Base
T_3 = vpa(T_2 * T_matrix(3).T, 2); % Elbow w.r.t. Base
T_4 = vpa(T_3 * T_matrix(4).T, 2); % Wrist 1 w.r.t. Base
T_5 = vpa(T_4 * T_matrix(5).T, 2); % Wrist 2 w.r.t. Base
T_6 = vpa(T_5 * T_matrix(6).T, 2); % TCP w.r.t. base


% rotation matrix, joint frames w.r.t. base
R_1 = T_1(1:3,1:3);
R_2 = T_2(1:3,1:3);
R_3 = T_3(1:3,1:3);
R_4 = T_4(1:3,1:3);
R_5 = T_5(1:3,1:3);
R_6 = T_6(1:3,1:3);

z0 = [0 0 1]';
z1 = T_1(1:3,3);
z2 = T_2(1:3,3);
z3 = T_3(1:3,3);
z4 = T_4(1:3,3);
z5 = T_5(1:3,3);
z6 = T_6(1:3,3);

% position of joint frames w.r.t. base
p0 = [0 0 0]';
p1 = T_1(1:3,4);
p2 = T_2(1:3,4);
p3 = T_3(1:3,4);
p4 = T_4(1:3,4);
p5 = T_5(1:3,4);
p6 = T_6(1:3,4);

J_p1_P1 = cross(z0,(p1-p0));
J_p2_P1 = cross(z0,(p2-p0));
J_p3_P1 = cross(z0,(p3-p0));
J_p4_P1 = cross(z0,(p4-p0));
J_p5_P1 = cross(z0,(p5-p0));
J_p6_P1 = cross(z0,(p6-p0));

J_p2_P2 = cross(z1,(p2-p1));
J_p3_P2 = cross(z1,(p3-p1));
J_p4_P2 = cross(z1,(p4-p1));
J_p5_P2 = cross(z1,(p5-p1));
J_p6_P2 = cross(z1,(p6-p1));

J_p3_P3 = cross(z2,(p3-p2));
J_p4_P3 = cross(z2,(p4-p2));
J_p5_P3 = cross(z2,(p5-p2));
J_p6_P3 = cross(z2,(p6-p2));

J_p4_P4 = cross(z3,(p4-p3));
J_p5_P4 = cross(z3,(p5-p3));
J_p6_P4 = cross(z3,(p6-p3));

J_p5_P5 = cross(z4,(p5-p4));
J_p6_P5 = cross(z4,(p6-p4));

J_p6_P6 = cross(z5,(p6-p5));

J_p1 = [J_p1_P1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p2 = [J_p2_P1    J_p2_P2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p3 = [J_p3_P1    J_p3_P2    J_p3_P3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_p4 = [J_p4_P1    J_p4_P2    J_p4_P3    J_p4_P4 zeros(3,1) zeros(3,1)];
J_p5 = [J_p5_P1    J_p5_P2    J_p5_P3    J_p5_P4    J_p5_P5 zeros(3,1)];
J_p6 = [J_p6_P1    J_p6_P2    J_p6_P3    J_p6_P4    J_p6_P5    J_p6_P6];

q   = [q1; q2; q3; q4; q5; q6];
dq  = [dq1; dq2; dq3; dq4; dq5; dq6];
ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6];

dp_1 = J_p1*dq;
dp_2 = J_p2*dq;
dp_3 = J_p3*dq;
dp_4 = J_p4*dq;
dp_5 = J_p5*dq;
dp_6 = J_p6*dq;

J_p1_O1 = z0;
J_p2_O1 = z0;
J_p3_O1 = z0;
J_p4_O1 = z0;
J_p5_O1 = z0;
J_p6_O1 = z0;

J_p2_O2 = z1;
J_p3_O2 = z1;
J_p4_O2 = z1;
J_p5_O2 = z1;
J_p6_O2 = z1;

J_p3_O3 = z2;
J_p4_O3 = z2;
J_p5_O3 = z2;
J_p6_O3 = z2;

J_p4_O4 = z3;
J_p5_O4 = z3;
J_p6_O4 = z3;

J_p5_O5 = z4;
J_p6_O5 = z4;

J_p6_O6 = z5;

J_p1_O = [J_p1_O1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p2_O = [J_p2_O1    J_p2_O2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p3_O = [J_p3_O1    J_p3_O2    J_p3_O3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_p4_O = [J_p4_O1    J_p4_O2    J_p4_O3    J_p4_O4 zeros(3,1) zeros(3,1)];
J_p5_O = [J_p5_O1    J_p5_O2    J_p5_O3    J_p5_O4    J_p5_O5 zeros(3,1)];
J_p6_O = [J_p6_O1    J_p6_O2    J_p6_O3    J_p6_O4    J_p6_O5    J_p6_O6];

w_1 = J_p1_O*dq;
w_2 = J_p2_O*dq;
w_3 = J_p3_O*dq;
w_4 = J_p4_O*dq;
w_5 = J_p5_O*dq;
w_6 = J_p6_O*dq;


S_w1 = [0 -w_1(3) w_1(2);
        w_1(3) 0 -w_1(1);
       -w_1(2) w_1(1) 0];

S_w2 = [0 -w_2(3) w_2(2);
        w_2(3) 0 -w_2(1);
       -w_2(2) w_2(1) 0];

S_w3 = [0 -w_3(3) w_3(2);
        w_3(3) 0 -w_3(1);
       -w_3(2) w_3(1) 0];

S_w4 = [0 -w_4(3) w_4(2);
        w_4(3) 0 -w_4(1);
       -w_4(2) w_4(1) 0];

S_w5 = [0 -w_5(3) w_5(2);
        w_5(3) 0 -w_5(1);
       -w_5(2) w_5(1) 0];

S_w6 = [0 -w_6(3) w_6(2);
        w_6(3) 0 -w_6(1);
       -w_6(2) w_6(1) 0];

T0 = 1/2*k_r1^2*dq1^2*I_m1; %T0 = simple(T0);
T1 = 1/2*m1*(dp_1'*dp_1) + dp_1'*S_w1*m1_rC1 + 1/2*w_1'*R_1*I_1_h*R_1'*w_1 + k_r2*dq2*I_m2*z2'*w_1 + 1/2*k_r2^2*dq2^2*I_m2; %T1 = simple(T1);
T2 = 1/2*m2*(dp_2'*dp_2) + dp_2'*S_w2*m2_rC2 + 1/2*w_2'*R_2*I_2_h*R_2'*w_2 + k_r3*dq3*I_m3*z3'*w_2 + 1/2*k_r3^2*dq3^2*I_m3; %T2 = simple(T2);
T3 = 1/2*m3*(dp_3'*dp_3) + dp_3'*S_w3*m3_rC3 + 1/2*w_3'*R_3*I_3_h*R_3'*w_3 + k_r4*dq4*I_m4*z4'*w_3 + 1/2*k_r4^2*dq4^2*I_m4; %T3 = simple(T3);
T4 = 1/2*m4*(dp_4'*dp_4) + dp_4'*S_w4*m4_rC4 + 1/2*w_4'*R_4*I_4_h*R_4'*w_4 + k_r5*dq5*I_m5*z5'*w_4 + 1/2*k_r5^2*dq5^2*I_m5; %T4 = simple(T4);
T5 = 1/2*m5*(dp_5'*dp_5) + dp_5'*S_w5*m5_rC5 + 1/2*w_5'*R_5*I_5_h*R_5'*w_5 + k_r6*dq6*I_m6*z6'*w_5 + 1/2*k_r6^2*dq6^2*I_m6;
T6 = 1/2*m6*(dp_6'*dp_6) + dp_6'*S_w6*m6_rC6 + 1/2*w_6'*R_6*I_6_h*R_6'*w_6;

T = T0 + T1 + T2 + T3 + T4 + T5 + T6;

g_0 = [0 0 -g]';
U1 = -g_0'*(m1*p1 + m1_rC1);
U2 = -g_0'*(m2*p2 + m2_rC2);
U3 = -g_0'*(m3*p3 + m3_rC3);
U4 = -g_0'*(m4*p4 + m4_rC4);
U5 = -g_0'*(m5*p5 + m5_rC5);
U6 = -g_0'*(m6*p6 + m6_rC6);

U = U1 + U2 + U3 + U4 + U5 + U6;

L = T-U;

disp('differentiating jacobian');
L_dq   = jacobian(L, [dq1 dq2 dq3 dq4 dq5 dq6]).';           % ∂L/∂dq
state  = [q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6].';      % [q; dq]
dstate = [dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6].';
L_dq_t = jacobian(L_dq, state) * dstate;                     % d/dt(∂L/∂dq)
L_q    = jacobian(L, [q1 q2 q3 q4 q5 q6]).';                 % ∂L/∂q
tau_sym_lagrangian = L_dq_t - L_q;   % τ = d/dt(∂L/∂dq) - ∂L/∂q

disp("saving the data into .mat files.");
save ../data/mat/tau_q_qd_qdd_sym tau_sym_lagrangian

disp("saving analytical torque as a function...");
qSym   = sym('q',[6,1]);
dqSym  = sym('dq',[6,1]);
ddqSym = sym('ddq',[6,1]);
matlabFunction(tau_sym_lagrangian, 'File','tau_func','Vars',{qSym,dqSym,ddqSym});


dt = mean(diff(tTraj));                   % seconds
qdTraj  = [zeros(1,6); diff(qTraj)/dt];   % velocity: [N x 6]
qddTraj = [zeros(1,6); diff(qdTraj)/dt];  % acceleration: [N x 6]

% inverse dynamics torques
analytical_tau = zeros(size(qTraj));
for i = 1:length(tTraj)
    q   = qTraj(i,:);
    qd  = qdTraj(i,:);
    qdd = qddTraj(i,:);
    
    analytical_tau(i,:) = tau_func(q', qd', qdd');
end

figure(Name="analytical torque");
for j = 1:6
    subplot(3,2,j);
    plot(tTraj, analytical_tau(:,j),'LineWidth',1.2);
    grid on;
    title(sprintf('Joint %d Torque',j));
    xlabel('Time [s]');
    ylabel(sprintf('\\tau_{%d} [Nm]', j));
end
sgtitle('Analytical Joint Torques')

save(analyticalWorkspace)

toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Function Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = dhTransform(a, alpha, d, theta)
    % dhTransform: returns 4×4 homogeneous transform of frame i relative
    % to frame i-1
    % a     – link length
    % alpha – link twist
    % d     – link offset
    % theta – joint angle

    ct = cos(theta);   st = sin(theta);
    ca = cos(alpha);   sa = sin(alpha);

    T = [ ct,   -st*ca,   st*sa,   a*ct;
          st,    ct*ca,  -ct*sa,   a*st;
          0,     sa,      ca,      d;
          0,     0,       0,       1 ];
    T = vpa(T, 4);
end

function [hdr, t, q] = readCSVFile(csvFile)

    % --- read everything (header + data) -------------------------------
    C     = readcell(csvFile);        % cell array with header row
    hdr   = C(1,:);                   % 1×N header row
    data  = cell2mat(C(2:end,:));     % numeric part (Ns × N)

    % --- split into variables -----------------------------------------
    t      = data(:,1);               % time column
    q      = data(:,  2:end);         % actual q

end