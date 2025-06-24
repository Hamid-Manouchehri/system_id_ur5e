%{
*******************************************************************************
Project: system-id ur5e
File: plot_mat.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 26, 2025

Description:
Script for plotting from .mat files.

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

clc; clear; close all

syms q1 q2 q3 q4 q5 q6 ...
     dq1 dq2 dq3 dq4 dq5 dq6 ...
     ddq1 ddq2 ddq3 ddq4 ddq5 ddq6

S = load('../data/mat/tau_lagrangian.mat');  % TODO

csv_file_name = 'ur5e_data_swing_elbow_100.csv';  % TODO, last number is logging freq
csv_dir = '/home/hamid/projects/system_id_ur5e/data/logs/';

fullfile = fullfile(csv_dir, csv_file_name);

% 2) Read into a table (handles headers automatically):
T = readtable(fullfile);

% 3) Convert to array for easy indexing:
data = table2array(T);

time           = data(:,1);      % timestamp in s
qs             = data(:,2:7);    % actual_q, [q₁,…,q₆] in rad
qds            = data(:,8:13);   % actual_qd, [q̇₁,…,q̇₆] in rad/s
qdds           = data(:,14:19);  % target_qdd, [qdd1, qdd2, ..., qdd6] in rad/s^2
tcpPose        = data(:,20:25);  % actual_TCP_pose, [x,y,z, Rx, Ry, Rz]
tcpSpeed       = data(:,26:31);  % actual_TCP_speed, [vx, vy, vz, ωx, ωy, ωz]
qActualCurrent = data(:,32:37);  % actual_current, [I1, I2, …, I6] in mA
qOutputCurrent = data(:,38:43);  % joint_control_output, [I1, I2, …, I6] in mA
tcpForce       = data(:,44:49);  % actual_TCP_force, [Fx, Fy, Fz, Tx, Ty, Tz]
targetMoment   = data(:,50:55);  % target_moment, [T1, T2, ..., T6] in Nm
qTemperature   = data(:,56:end); % joint_temperatures [t1, t2, ..., t6] in degrees Celsius

num_samples = 200;  % TODO

row_idx = round(linspace(1, size(qs,1), num_samples));  % indices from 1 to 1000
sample_time = time(row_idx, :); 
sampled_q = qs(row_idx, :);
sampled_qd = qds(row_idx, :); 
sampled_qdd = qdds(row_idx, :); 

% qs = [0., -1.57, 0., -1.57, 0., 0.];  % joint: [base, shoulder, elbow, w1, w2, w3]
% dqs = [1, 1, 1, 1, 1, 1] * 0.5;
% ddqs = [1, 1, 1, 1, 1, 1] * 0.5;

tau_hat = zeros(num_samples,6);

for j = 1:num_samples
    for i = 1:6
        % substitute all 6 torque expressions at once:
        vals = double( subs( S.tau_lagrangian(i), ...
            [q1 q2 q3 q4 q5 q6, ...
             dq1 dq2 dq3 dq4 dq5 dq6, ...
             ddq1 ddq2 ddq3 ddq4 ddq5 ddq6], ...
            [sampled_q(j,:),  sampled_qd(j,:),  sampled_qdd(j,:)] ) );
        tau_hat(j,i) = vals;
    end
    disp(j)
end

figure;
plot(sample_time, tau_hat, 'LineWidth', 1.5);    % Each column = one joint
xlabel('Sample');
ylabel('Torque (Nm)');
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6');
title('Estimated Torques (\tau_{hat}) for 6 Joints');
grid on;
