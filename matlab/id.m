%{
*******************************************************************************
Project: system-id ur5e
File: id.m
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 23, 2025

Description:
Script for symbolic calculation of kinematics and dynamics of UR5e robot.

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

m = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

% center of mass [m]
m1_lC1 = [0.0, 0.02561, 0.00193];
m2_lC2 = [0.2125, 0.0, 0.11336];
m3_lC3 = [0.15, 0.0, 0.0265];
m4_lC4 = [0.0, -0.0018, 0.01634];
m5_lC5 = [0.0, 0.0018, 0.01634];
m6_lC6 = [0.0, 0.0, -0.001159];

% motor inertia
% I_m = [6.6119 4.6147 7.9773 1.2249 1.1868 1.1981]*1e-5;

% link inertia [I_xx I_yy I_zz I_xy I_yz I_xz] [kgm^2]
% I_h1 = [ 0.0067, 0.0064, 0.0067, 0.0, 0.0, 0.0];
% I_h2 = [ 0.0149, 0.3564, 0.3553, 0.0, 0.0, 0.0];
% I_h3 = [ 0.0025, 0.0551, 0.0546, 0.0, 0.0, 0.0034];
% I_h4 = [ 0.0012, 0.0012, 0.0009, 0.0, 0.0, 0.0];
% I_h5 = [ 0.0012, 0.0012, 0.009, 0.0, 0.0, 0.0];
% I_h6 = [ 0.001, 0.001, 0.001, 0.0, 0.0, 0.0];

syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ... % Joint State
    m1 m2 m3 m4 m5 m6 ... % mi = m_li + m_m_i+1
    I_h1xx I_h1xy I_h1xz I_h1yy I_h1yz I_h1zz ... % Steiner Theorem : I_hixx = I_li + m_li*S(r_Ci,li)'*S(r_Ci,li) + m_i*(l_Ciy^2 + l_Ciz^2)
    I_h2xx I_h2xy I_h2xz I_h2yy I_h2yz I_h2zz ...
    I_h3xx I_h3xy I_h3xz I_h3yy I_h3yz I_h3zz ...
    I_h4xx I_h4xy I_h4xz I_h4yy I_h4yz I_h4zz ...
    I_h5xx I_h5xy I_h5xz I_h5yy I_h5yz I_h5zz ...
    I_h6xx I_h6xy I_h6xz I_h6yy I_h6yz I_h6zz ...
    m1_lC1x m1_lC1y m1_lC1z m2_lC2x m2_lC2y m2_lC2z m3_lC3x m3_lC3y m3_lC3z ... % page 262 _ eq 7.72 _ fig 7.13
    m4_lC4x m4_lC4y m4_lC4z m5_lC5x m5_lC5y m5_lC5z m6_lC6x m6_lC6y m6_lC6z...
    I_m1 I_m2 I_m3 I_m4 I_m5 I_m6 real;

qs = [q1 q2 q3 q4 q5 q6];

p = [m1 m1_lC1x m1_lC1y m1_lC1z I_h1xx I_h1xy I_h1xz I_h1yy I_h1yz I_h1zz I_m1 ...
     m2 m2_lC2x m2_lC2y m2_lC2z I_h2xx I_h2xy I_h2xz I_h2yy I_h2yz I_h2zz I_m2 ...
     m3 m3_lC3x m3_lC3y m3_lC3z I_h3xx I_h3xy I_h3xz I_h3yy I_h3yz I_h3zz I_m3 ...
     m4 m4_lC4x m4_lC4y m4_lC4z I_h4xx I_h4xy I_h4xz I_h4yy I_h4yz I_h4zz I_m4 ...
     m5 m5_lC5x m5_lC5y m5_lC5z I_h5xx I_h5xy I_h5xz I_h5yy I_h5yz I_h5zz I_m5 ...
     m6 m6_lC6x m6_lC6y m6_lC6z I_h6xx I_h6xy I_h6xz I_h6yy I_h6yz I_h6zz I_m6]';

m1_lC1x = 0;
m1_lC1z = 0;
m2_lC2y = 0;
m3_lC3x = 0;
m4_lC4x = 0;
m5_lC5x = 0;
m6_lC6x = 0;
m6_lC6y = 0;
I_h1xy = 0; I_h1xz = 0; I_h1yz = 0;
I_h2xy = 0; I_h2yz = 0;
I_h3xy = 0; I_h3xz = 0;
I_h4xy = 0; I_h4xz = 0;
I_h5xy = 0; I_h5xz = 0;
I_h6xy = 0; I_h6xz = 0; I_h6yz = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m1_rC1 = [m1_lC1x m1_lC1y m1_lC1z]';
m2_rC2 = [m2_lC2x m2_lC2y m2_lC2z]';
m3_rC3 = [m3_lC3x m3_lC3y m3_lC3z]';
m4_rC4 = [m4_lC4x m4_lC4y m4_lC4z]';
m5_rC5 = [m5_lC5x m5_lC5y m5_lC5z]';
m6_rC6 = [m6_lC6x m6_lC6y m6_lC6z]';

I_1_h = [I_h1xx -I_h1xy -I_h1xz;
        -I_h1xy I_h1yy -I_h1yz;
        -I_h1xz -I_h1yz I_h1zz];

I_2_h = [I_h2xx -I_h2xy -I_h2xz;
        -I_h2xy I_h2yy -I_h2yz;
        -I_h2xz -I_h2yz I_h2zz];

I_3_h = [I_h3xx -I_h3xy -I_h3xz;
        -I_h3xy I_h3yy -I_h3yz;
        -I_h3xz -I_h3yz I_h3zz];

I_4_h = [I_h4xx -I_h4xy -I_h4xz;
        -I_h4xy I_h4yy -I_h4yz;
        -I_h4xz -I_h4yz I_h4zz];

I_5_h = [I_h5xx -I_h5xy -I_h5xz;
        -I_h5xy I_h5yy -I_h5yz;
        -I_h5xz -I_h5yz I_h5zz];

I_6_h = [I_h6xx -I_h6xy -I_h6xz;
        -I_h6xy I_h6yy -I_h6yz;
        -I_h6xz -I_h6yz I_h6zz];

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

% f_sub = subs(T_tcp_to_base, qs, [1.0, -2.57, 0.0, -2.57, 0.3, 0.0]);
% f_val = double(f_sub);
% pose = f_val(1:3,4)
% orientation = f_val(1:3,1:3)

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

q = [q1 q2 q3 q4 q5 q6]';
dq = [dq1 dq2 dq3 dq4 dq5 dq6]';
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6]';

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

state = [q' dq']';
dstate = [dq' ddq']';

L = T-U;

disp('differentiating jacobian');
L_dq = jacobian(L,dq)';
L_dq_t = jacobian(L_dq,state)*dstate;
L_q = jacobian(L,q)';
tau_lagrangian = L_dq_t - L_q;  % analytical dynamic formulation
Y_sym = jacobian(tau_lagrangian,p);  % regressor: Y(q, dq, ddq)

disp("saving the data into .mat files.");
save ../data/mat/tau_lagrangian tau_lagrangian
save ../data/mat/Y_sym Y_sym
save ../data/mat/p p

disp("saving Y_sym as a function...");
qSym   = sym('q',[6,1]);
dqSym  = sym('dq',[6,1]);
ddqSym = sym('ddq',[6,1]);
matlabFunction(Y_sym, 'File','Y_fun','Vars',{qSym,dqSym,ddqSym});


% [q, qd, qdd] = optimization_fcn(Y);

% syms q [1 6] dq [1 6] ddq [1 6]
% Y_sym = my_regressor(q, dq, ddq);  % Or however you define it

% lambda1 = 1;
% lambda2 = 1;
% state0 = zeros(18,1);         % Initial guess for 18 variables
% A = []; b = [];
% Aeq = []; beq = [];
% qMax = 2*pi * ones(6,1);
% qdMax = pi * ones(6,1);
% qddMax = 5 * ones(6,1);
% state_max = [qMax; qdMax; qddMax];
% ub = state_max; lb = -state_max;
% 
% disp("objective fun");
% obj_fcn = @(x) objective_fun(x, Y_sym, lambda1, lambda2);
% 
% disp("fmincon");
% opts = optimoptions('fmincon','Display','iter','Algorithm','sqp');
% state_opt = fmincon(obj_fcn, state0, A, b, Aeq, beq, lb, ub, [], opts);
% state0 = state_opt;




% N          = 4;
% duration_s = 1;
% dt         = 0.1;
% t          = linspace(0, duration_s, duration_s/dt);
% omega_f    = pi / 5;
% 
% 
% % initial guess (e.g. all ones)
% A0 = ones(N,1);
% B0 = ones(N,1);
% x0 = [A0; B0];
% 
% A = []; b = [];
% Aeq = []; beq = [];
% % bounds: 1 < a_i < 2,   3 < b_i < 5
% lb = [1*ones(N,1) ; 3*ones(N,1)];
% ub = [2*ones(N,1) ; 5*ones(N,1)];
% 
% q0s = [0. 0. 0. 0. 0. 0.];
% dq0s = [0. 0. 0. 0. 0. 0.];
% ddq0s = [0. 0. 0. 0. 0. 0.];
% state0 = [q0s dq0s ddq0s];
% 
% states_sym = [q1 q2 q3 q4 q5 q6 ...
%               dq1 dq2 dq3 dq4 dq5 dq6 ...
%               ddq1 ddq2 ddq3 ddq4 ddq5 ddq6];
% 
% 
% opts = optimoptions('fmincon', 'Algorithm', 'sqp');
% 
% % Preallocate storage for the optimal Fourier‐coeffs at each t_k
% X_opt = zeros(2*N, numel(t));
% 
% for k = 1:numel(t)
%     tk = t(k);
% 
%     % build a handle that only cares about this single time tk
%     obj_at_tk = @(x) cond_at_time(x, tk, state0, N, Y_sym, states_sym, omega_f);
% 
%     % solve the small 2N-var problem for this tk
%     X_opt(:,k) = fmincon(obj_at_tk, state0, A, b, Aeq, beq, lb, ub, [], opts);
%     disp(["time:", tk, "fmincon:", X_opt(:, k)'])
% end












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


function cost = objective_fun(x, Y_sym, lambda1, lambda2)
    % x: 18x1 vector [q1:6, dq1:6, ddq1:6]
    % Substitute x into symbolic regressor
    q_syms   = sym('q',   [1 6]);
    dq_syms  = sym('dq',  [1 6]);
    ddq_syms = sym('ddq', [1 6]);
    all_syms = [q_syms, dq_syms, ddq_syms];
    
    x(:)

    Y_num = double(subs(Y_sym, all_syms, x(:)'));
    
    % Penalize singular/invalid matrices
    if any(isnan(Y_num(:))) || rank(Y_num) < min(size(Y_num))
        cost = 1e6;
        return;
    end
    condition_num = cond(Y_num);
    smallest_sv = min(svd(Y_num));
    cost = lambda1 * condition_num + lambda2 / smallest_sv;
end


function Q = fourier_all_joints(t, A, B, q0)
    % Q: [length(t) x 6], each column is one joint's q(t)
    omega_f = 2*pi/10;
    if nargin < 5
        q0 = zeros(6,1);
    end
    N = size(A,2);
    T = t(:);              % column vector of times
    L = 1:N;
    WFL = omega_f * L;     % 1 x N

    Q = zeros(length(T), 6);
    for i = 1:6
        % Broadcast to all t
        terms = zeros(length(T), 1);
        for l = 1:N
            terms = terms + (A(i,l)/WFL(l)) * sin(WFL(l)*T) ...
                          - (B(i,l)/WFL(l)) * cos(WFL(l)*T);
        end
        Q(:,i) = terms + q0(i);
    end
end

function J = cond_at_time(x, tk, q_dq_ddq_0, N, Y_sym, q_dq_ddq_sym, omega_f)
    % x = [A(1:N); B(1:N)]
    % tk = scalar time
    % compute q1(tk) from the Fourier coefficients
    a = x(1:N);
    b = x(N+1:end);
    q   = q_dq_ddq_0(1:6);
    dq  = q_dq_ddq_0(7:12);
    ddq = q_dq_ddq_0(13:end);

    for i = 1:6
        for l = 1:N
            wfl = omega_f * l;
            q(i)   = q(i) + (a(l)/wfl)*sin(wfl*tk) - (b(l)/wfl)*cos(wfl*tk);
            dq(i)  = a(l)*cos(wfl*tk) + b(l)*sin(wfl*tk);
            ddq(i) = - a(l)*wfl*sin(wfl*tk) + b(l)*wfl*cos(wfl*tk);
        end
    end

    % substitute into the symbolic regressor and get cond
    Y = double( subs(Y_sym, q_dq_ddq_sym, [q dq ddq]) );
    s = svd(Y);

    % compute condition number and min singular value:
    condY = s(1)/s(end);
    minSignVal = s(end);
    J = condY + 1 / minSignVal
end

% function J = cond_at_time(x, tk, q0, N, Y_sym, q_dq_ddq_sym, omega_f)
%   % x         : [A(:); B(:)] with length 2*6*N
%   % tk        : scalar time sample
%   % q0        : 6×1 vector of initial offsets for q
%   % N         : number of harmonics
%   % Y_sym     : symbolic regressor (size m×n) in variables [q_syms,dq_syms,ddq_syms]
%   % q_syms    : 1×6 sym array [q1…q6]
%   % dq_syms   : 1×6 sym array [dq1…dq6]
%   % ddq_syms  : 1×6 sym array [ddq1…ddq6]
%   % omega_f   : fundamental frequency
% 
%   % unpack A and B into 6×N matrices
%   A_mat = reshape( x(              1:6*N), [6, N] );
%   B_mat = reshape( x(6*N + (1:6*N)), [6, N] );
% 
%   % pre-allocate
%   q   = q0(:).';               % 1×6
%   dq  = zeros(1,6);
%   ddq = zeros(1,6);
% 
%   % build q, dq, ddq at time tk
%   for i = 1:6
%     for l = 1:N
%       wfl = omega_f * l;
%       a = A_mat(i,l);
%       b = B_mat(i,l);
%       q(i)   = q(i)   + (a/wfl)*sin(wfl*tk)     - (b/wfl)*cos(wfl*tk);
%       dq(i)  = dq(i)  +     a *     cos(wfl*tk) +     b * sin(wfl*tk);
%       ddq(i) = ddq(i) + (-a*wfl * sin(wfl*tk))  + ( b*wfl * cos(wfl*tk));
%     end
%   end
% 
%   % substitute into the symbolic regressor
%   M_num = double( subs( Y_sym, q_dq_ddq_sym, [q, dq, ddq] ) );
% 
%   % compute singular values + cost
%   s = svd(M_num);
%   condY = s(1)/s(end);
%   J     = condY + 1/s(end);
% end
