%{
*******************************************************************************
Project: system-id ur5e
File: ik.py
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
I_m = [6.6119 4.6147 7.9773 1.2249 1.1868 1.1981]*1e-5;

% link inertia [I_xx I_yy I_zz I_xy I_yz I_xz] [kgm^2]
I_h1 = [ 0.0067, 0.0064, 0.0067, 0.0, 0.0, 0.0];
I_h2 = [ 0.0149, 0.3564, 0.3553, 0.0, 0.0, 0.0];
I_h3 = [ 0.0025, 0.0551, 0.0546, 0.0, 0.0, 0.0034];
I_h4 = [ 0.0012, 0.0012, 0.0009, 0.0, 0.0, 0.0];
I_h5 = [ 0.0012, 0.0012, 0.009, 0.0, 0.0, 0.0];
I_h6 = [ 0.001, 0.001, 0.001, 0.0, 0.0, 0.0];

syms a1 a2 a3 a4 a5 a6
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

%%%%%% Simplification %%%%%%%%
% m1_lC1x = 0;
% m1_lC1z = 0;
% m2_lC2y = 0;
% m3_lC3x = 0;
% m4_lC4x = 0;
% m5_lC5x = 0;
% m6_lC6x = 0;
% m6_lC6y = 0;
% I_h1xy = 0; I_h1xz = 0; I_h1yz = 0;
% I_h2xy = 0; I_h2yz = 0;
% I_h3xy = 0; I_h3xz = 0;
% I_h4xy = 0; I_h4xz = 0;
% I_h5xy = 0; I_h5xz = 0;
% I_h6xy = 0; I_h6xz = 0; I_h6yz = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m1_rC1 = [m1_lC1(1) m1_lC1(2) m1_lC1(3)]';
m2_rC2 = [m2_lC2(1) m2_lC2(2) m2_lC2(3)]';
m3_rC3 = [m3_lC3(1) m3_lC3(2) m3_lC3(3)]';
m4_rC4 = [m4_lC4(1) m4_lC4(2) m4_lC4(3)]';
m5_rC5 = [m5_lC5(1) m5_lC5(2) m5_lC5(3)]';
m6_rC6 = [m6_lC6(1) m6_lC6(2) m6_lC6(3)]';

I_1_h = [I_h1(1) -I_h1(4) -I_h1(6);
        -I_h1(4)  I_h1(2) -I_h1(5);
        -I_h1(6) -I_h1(5)  I_h1(3)];

I_2_h = [I_h2(1) -I_h2(4) -I_h2(6);
        -I_h2(4)  I_h2(2) -I_h2(5);
        -I_h2(6) -I_h2(5)  I_h2(3)];

I_3_h = [I_h3(1) -I_h3(4) -I_h3(6);
        -I_h3(4)  I_h3(2) -I_h3(5);
        -I_h3(6) -I_h3(5)  I_h3(3)];

I_4_h = [I_h4(1) -I_h4(4) -I_h4(6);
        -I_h4(4)  I_h4(2) -I_h4(5);
        -I_h4(6) -I_h4(5)  I_h4(3)];

I_5_h = [I_h5(1) -I_h5(4) -I_h5(6);
        -I_h5(4)  I_h5(2) -I_h5(5);
        -I_h5(6) -I_h5(5)  I_h5(3)];

I_6_h = [I_h6(1) -I_h6(4) -I_h6(6);
        -I_h6(4)  I_h6(2) -I_h6(5);
        -I_h6(6) -I_h6(5)  I_h6(3)];

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

p0 = [0 0 0]';
p1 = T_1(1:3,4);
p2 = T_2(1:3,4);
p3 = T_3(1:3,4);
p4 = T_4(1:3,4);
p5 = T_5(1:3,4);
p6 = T_6(1:3,4);


% qs_home = [0., -1.57, 0., -1.57, 0., 0.];  % joint: [base, shoulder, elbow, w1, w2, w3]
% vpa(subs(p6, qs, qs_home), 5)






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