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

joint_traj_file = 'ur5e_smooth_random_joint_traj_1.csv';  % TODO
Joint_traj_dir  = '../data/logs/';
csvFile = fullfile(Joint_traj_dir, joint_traj_file);

% [ur5e_data, header] = readmatrix(csvFile, 'OutputType','double');

C = readcell(csvFile);
header_ur5e_data = C(1,:);
ur5e_data = cell2mat(C(2:end,:));

logged_actual_q = ur5e_data(:,2:7);
logged_actual_qd = ur5e_data(:,8:13);
logged_target_qdd = ur5e_data(:,14:19);





% Y_fun_actual = Y_fun();
% Y_fun_ = Y_fun();