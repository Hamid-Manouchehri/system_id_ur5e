%{
*******************************************************************************
Project: system-id ur5e
File: robotic_system_toolbox.m
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: Aug 06, 2025

Description:
Basic example of inverse dynamics with matlab robotic-system-toolbox
for validation of dynamic of UR5e derived from lagrangian fromulation (id.m).

License:
This script is licensed under the MIT License.
You may obtain a copy of the License at
    https://opensource.org/l
icenses/MIT

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

trajFile = 'ur5e_smooth_random_joint_traj_v2.csv';
trajDir  = '../data/traj';
csvTraj = fullfile(trajDir, trajFile);
[hdr, tTraj, qTraj] = readCSVFile(csvTraj);

ur5e_model = loadrobot("universalUR5e");
ur5e_model.DataFormat = 'row';
ur5e_model.Gravity = [0 0 -9.81];
ur5eHomeConfig = homeConfiguration(ur5e_model);

qUpRight = [0., pi/2, 0., pi/2, 0., 0.];  % TODO

qUpRight(1) = ur5eHomeConfig(1) + qUpRight(1);
qUpRight(2) = ur5eHomeConfig(2) + qUpRight(2);
qUpRight(3) = ur5eHomeConfig(3) + qUpRight(3);
qUpRight(4) = ur5eHomeConfig(4) + qUpRight(4);
qUpRight(5) = ur5eHomeConfig(5) + qUpRight(5);
qUpRight(6) = ur5eHomeConfig(6) + qUpRight(6);

currentq = ur5eJointAnglesRad(ur5e_model, qUpRight);

% t_final = 10;
% fs = 100;
% t = linspace(0, t_final, t_final*fs)';

% create a smooth trajectory for 6 joints

% q1_traj = currentq(1) + sin(2*pi*0.1*t);
% q2_traj = currentq(2) + 0. * t;
% q3_traj = currentq(3) + 0. * t;
% q4_traj = currentq(4) + 0. * t;
% q5_traj = currentq(5) + 0. * t;
% q6_traj = currentq(6) + 0. * t;
% q_traj = [q1_traj q2_traj q3_traj q4_traj q5_traj q6_traj];
% 
% qd1_traj = 2*pi * 0.1 * cos(2*pi*0.1*t);
% qd2_traj = 0. * t;
% qd3_traj = 0. * t;
% qd4_traj = 0. * t;
% qd5_traj = 0. * t;
% qd6_traj = 0. * t;
% qd_traj = [qd1_traj qd2_traj qd3_traj qd4_traj qd5_traj qd6_traj];
% 
% qdd1_traj = -4*pi^2 * 0.01 * sin(2*pi*0.1*t);
% qdd2_traj = 0. * t;
% qdd3_traj = 0. * t;
% qdd4_traj = 0. * t;
% qdd5_traj = 0. * t;
% qdd6_traj = 0. * t;
% qdd_traj = [qdd1_traj qdd2_traj qdd3_traj qdd4_traj qdd5_traj qdd6_traj];

dt = mean(diff(tTraj));                   % seconds
qdTraj  = [zeros(1,6); diff(qTraj)/dt];        % velocity: [N x 6]
qddTraj = [zeros(1,6); diff(qdTraj)/dt];       % acceleration: [N x 6]

% inverse dynamics torques
tau = zeros(size(qTraj));
for i = 1:length(tTraj)
    q   = qTraj(i,:);
    qd  = qdTraj(i,:);
    qdd = qddTraj(i,:);
    
    tau(i,:) = inverseDynamics(ur5e_model, q, qd, qdd);
end

% figure;
% plot(tTraj, qTraj)
% plot(tTraj, qdTraj)
% plot(tTraj, qddTraj)

% Plot joint torques
figure(1);
for j = 1:6
    subplot(3,2,j);
    plot(tTraj, tau(:,j),'LineWidth',1.2);
    grid on;
    title(sprintf('Joint %d Torque',j));
    xlabel('Time [s]');
    ylabel(sprintf('\\tau_{%d} [Nm]', j));
end
sgtitle('UR5e Matlab Sim Joint Torques')

% Visual animation of UR5e trajectory:
figure(2);
for k = 1:5:length(tTraj)
    show(ur5e_model, qTraj(k,:), 'PreservePlot',false,'Frames','off');
    title(sprintf('UR5e Simulation (t=%.2fs)', tTraj(k)));
    drawnow;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Function Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [hdr, t, q] = readCSVFile(csvFile)

    % --- read everything (header + data) -------------------------------
    C     = readcell(csvFile);        % cell array with header row
    hdr   = C(1,:);                   % 1×N header row
    data  = cell2mat(C(2:end,:));     % numeric part (Ns × N)

    % --- split into variables -----------------------------------------
    t      = data(:,1);               % time column
    q      = data(:,  2:end);         % actual q

end


function q = ur5eJointAnglesRad(robot, configuration)

    T01 = getTransform(robot, configuration, 'base_link_inertia', 'shoulder_link');
    q1 = rotm2eul(T01(1:3,1:3), "ZYX");

    T12 = getTransform(robot, configuration, 'shoulder_link', 'upper_arm_link');
    q2 = rotm2eul(T12(1:3,1:3), "ZYX");

    T23 = getTransform(robot, configuration, 'upper_arm_link', 'forearm_link');
    q3 = rotm2eul(T23(1:3,1:3), "ZYX");

    T34 = getTransform(robot, configuration, 'forearm_link', 'wrist_1_link');
    q4 = rotm2eul(T34(1:3,1:3), "ZYX");

    T45 = getTransform(robot, configuration, 'wrist_1_link', 'wrist_2_link');
    q5 = rotm2eul(T45(1:3,1:3), "ZYX");

    T56 = getTransform(robot, configuration, 'wrist_2_link', 'wrist_3_link');
    q6 = rotm2eul(T56(1:3,1:3), "ZYX");

    q = [q1(1), q2(1), q3(1), q4(1), q5(1), q6(1)];

end