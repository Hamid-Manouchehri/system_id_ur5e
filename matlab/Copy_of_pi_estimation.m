%{
*******************************************************************************
Project: system-id ur5e
File: pi_estimation.m
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: July 25, 2025

Description:
Script for estimating pi (tau = Y(q,qd,qdd)*pi) according to the generated 
trajectory.

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

% Load the symbolic regressor
tmp   = load('../data/mat/Y_sym.mat','Y_sym');
Y_sym = tmp.Y_sym;
Y_fun = @Y_fun;  % Y_fun(q', dq', ddq'); q, dq, and ddq are column vectors

joint_traj_file_1 = 'ur5e_smooth_random_joint_traj_1.csv';  % TODO
joint_traj_file_2 = 'ur5e_smooth_random_joint_traj_2.csv';  % TODO
joint_traj_file_3 = 'ur5e_smooth_random_joint_traj_3.csv';  % TODO
Joint_traj_dir  = '../data/logs/';
actualTraj_csvFile_1 = fullfile(Joint_traj_dir, joint_traj_file_1);
actualTraj_csvFile_2 = fullfile(Joint_traj_dir, joint_traj_file_2);
actualTraj_csvFile_3 = fullfile(Joint_traj_dir, joint_traj_file_3);




[hdr, t_1, actual_q_1, actual_qd_1, actualTarget_qdd_1, actualTau_1] = ...
    readUR5eLog(actualTraj_csvFile_1);
Ybig_1 = stackRegressor(actual_q_1, actual_qd_1, actualTarget_qdd_1, Y_fun);
% pi_hat_1 = Ybig_1 \ actualTau_1;  % Estimated dynamic parameters
pi_hat_1 = pinv(Ybig_1) * actualTau_1;  % Estimated dynamic parameters

% [theta_est2,resnorm,residual,exitflag,output,lambda] = lsqlin(Y_meas,taw_pure,[],[],[],[],lb,ub,[],[]);

figure(1)
subplot(4,1,1); plot(t_1, actual_q_1); ylabel("actual q_i");
subplot(4,1,2); plot(t_1, actual_qd_1); ylabel("actual qd_i");
subplot(4,1,3); plot(t_1, actualTarget_qdd_1); ylabel("actual target qdd_i");
actualTau_rec_1 = reshape(actualTau_1, 6, []).';  % Reconstructed torque matrix
% subplot(4,1,4); plot(t_1, actualTau_rec_1); ylabel("actual \tau");

smoothActualTauMat_rec_1 = zeros(size(actualTau_rec_1, 1), size(actualTau_rec_1, 2));
for i=1:6
    smoothActualTauMat_rec_1(:,i) = smooth(actualTau_rec_1(:,i));
end
subplot(4,1,4); plot(t_1, smoothActualTauMat_rec_1); ylabel("actual \tau");

tau_error_1 = actualTau_1 - (Ybig_1 * pi_hat_1);
tau_error_rec_1 = reshape(tau_error_1, 6, []).';
figure
plot(t_1, tau_error_rec_1); ylabel("error \tau");




[~, t_2, actual_q_2, actual_qd_2, actualTarget_qdd_2, actualTau_2] = ...
    readUR5eLog(actualTraj_csvFile_2);
Ybig_2 = stackRegressor(actual_q_2, actual_qd_2, actualTarget_qdd_2, Y_fun);
tau_est_2 = Ybig_2 * pi_hat_1;
error_tau_2 = actualTau_2 - tau_est_2;
tauMat_rec_2 = reshape(error_tau_2, 6, []).';  % Reconstructed torque matrix

smoothTauMat_rec_2 = zeros(length(t_2),6);
for i=1:6
    smoothTauMat_rec_2(:,i) = smooth(tauMat_rec_2(:,i));
end

figure(2);
subplot(3,2,1); plot(t_2,smoothTauMat_rec_2(:,1)); ylabel("\tau_1 error");
subplot(3,2,2); plot(t_2,smoothTauMat_rec_2(:,2)); ylabel("\tau_2 error");
subplot(3,2,3); plot(t_2,smoothTauMat_rec_2(:,3)); ylabel("\tau_3 error");
subplot(3,2,4); plot(t_2,smoothTauMat_rec_2(:,4)); ylabel("\tau_4 error");
subplot(3,2,5); plot(t_2,smoothTauMat_rec_2(:,5)); ylabel("\tau_5 error");
subplot(3,2,6); plot(t_2,smoothTauMat_rec_2(:,6)); ylabel("\tau_6 error");



real_tau = reshape(actualTau_1, 6, []).';
ext_tau  = reshape(actual_q_2, 6, []).';

figure(3);
for i = 1:6
    subplot(3,2,i); 
    plot(real_tau(:,i),"r-."); ylabel(sprintf('\\tau_{%d}', i)); 
    hold on;
    plot(ext_tau(:,i),"b-.");
end





[~, t_3, actual_q_3, actual_qd_3, actualTarget_qdd_3, actualTau_3] = ...
    readUR5eLog(actualTraj_csvFile_3);
Ybig_3 = stackRegressor(actual_q_3, actual_qd_3, actualTarget_qdd_3, Y_fun);
tau_est_3 = Ybig_3 * pi_hat_1;
error_tau_3 = actualTau_3 - tau_est_3;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Function Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [hdr, t, q, qd, qdd, tau] = readUR5eLog(csvFile)
%READUR5ELOG  Import a UR5e joint-log CSV created by your script.
%
%   [hdr, t, q, qd, qdd, tau] = READUR5ELOG(csvFile) reads the CSV whose
%   path is given by csvFile and returns:
%
%     hdr   – 1×N cell array   : header row (column names)
%     t     – Ns×1 double      : time stamps  (s)
%     q     – Ns×6 double      : actual joint positions  (rad)
%     qd    – Ns×6 double      : actual joint velocities (rad/s)
%     qdd   – Ns×6 double      : target joint accelerations (rad/s²)
%     tau   – Ns*6 ×1 double   : actual joint torques, flattened column-wise
%
%   The column layout is assumed to be exactly the same as in your
%   original script (cols 1,2-7,8-13,14-19,62-end).
%
%   Example:
%       file = fullfile('..','data','logs','ur5e_smooth_random_joint_traj_1.csv');
%       [hdr,t,q,qd,qdd,tau] = readUr5eLog(file);

    % --- read everything (header + data) -------------------------------
    C     = readcell(csvFile);        % cell array with header row
    hdr   = C(1,:);                   % 1×N header row
    data  = cell2mat(C(2:end,:));     % numeric part (Ns × N)

    % --- split into variables -----------------------------------------
    t      = data(:,1);               % time column
    q      = data(:,  2:  7);         % actual q
    qd     = data(:,  8: 13);         % actual qd
    qdd    = data(:, 14: 19);         % target qdd
    tauMat = data(:, 62:end);         % actual torque (Ns × 6)

    % Flatten torques column-wise to (Ns*6)×1
    tau = reshape(tauMat.', [], 1);
end


function Ybig = stackRegressor(q, qd, qdd, Y_fun)
%STACKREGRESSOR  Build the stacked regressor for a whole trajectory.
%
%   Ybig = STACKREGRESSOR(q, qd, qdd, Y_fun)
%
%   INPUTS
%     q    – Ns×6  joint positions           [rad]
%     qd   – Ns×6  joint velocities          [rad/s]
%     qdd  – Ns×6  joint accelerations       [rad/s²]
%     Y_fun– handle to regressor, signature Y = Y_fun(qCol, qdCol, qddCol)
%            where each argument is 6×1 (column vector) and
%            Y has size 6×P.
%
%   OUTPUT
%     Ybig – (Ns*6) × P  stacked regressor (block-diagonal), i.e.
%             [ Y(q1) ]
%             [ Y(q2) ]
%             [   …   ]
%             [ Y(qNs)]
%
%   --------------------------------------------------------------------
%   Example:
%       Ybig = stackRegressor(actual_q_1, actual_qd_1, actualTarget_qdd_1, Y_fun);
%
%   --------------------------------------------------------------------

    Ns = size(q, 1);                     % number of samples
    P  = size(Y_fun(q(1,:)',qd(1,:)',qdd(1,:)'), 2);  % regressor width

    Ybig = zeros(Ns*6, P);               % pre-allocate

    for k = 1:Ns
        qCol   = q(k, :)';               % 6×1
        qdCol  = qd(k, :)';
        qddCol = qdd(k, :)';

        idx = (k-1)*6 + (1:6);           % row block for this sample
        Ybig(idx, :) = Y_fun(qCol, qdCol, qddCol);
    end
end
