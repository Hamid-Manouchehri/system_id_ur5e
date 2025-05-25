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

syms q1 q2 q3 q4 q5 q6
syms a1 a2 a3 a4 a5 a6

qs = [q1 q2 q3 q4 q5 q6];

TCP_offset_z = 0.171; % m

% UR5e DH parameters:
a     = [0.0 -0.425 -0.3922 0.0 0.0 0.0];
alpha = [pi/2 0.0 0.0 pi/2 -pi/2 0.0];
d     = [0.1625 0.0 0.0 0.1333 0.0977 0.0966 + TCP_offset_z];

% Clculating symbolic transformation matrices for all links:
T_matrix = struct('T', cell(1,6));
T_tcp_to_base = eye(4);

for i = 1:6
    % homogeneous transform of frame i relative to frame i-1
    T_matrix(i).T = dhTransform(a(i), alpha(i), d(i), qs(i));
    T_tcp_to_base = T_end_to_base * T_matrix(i).T;
end

f_sub = subs(T_end_to_base, qs, [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]);
f_val = double(f_sub)

pose = f_val(1:3,4)
orientation = f_val(1:3,1:3)





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