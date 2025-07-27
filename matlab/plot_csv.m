%{
*******************************************************************************
Project: system-id ur5e
File: plot_csv.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: May 11, 2025

Description:
Script for plotting ur5e states from csv files.

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

csv_file_name = 'ur5e_smooth_random_joint_traj_v1.csv';  % TODO, 10 is logging freq
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








plotJointPositions(time, qs);
plotJointVelocities(time, qds);
plotTargetJointAccelerations(time, qdds);
% plotTCPPose(time, tcpPose);
% plotTCPSpeed(time, tcpSpeed);
plotJointActualCurrents(time, qActualCurrent);
plotJointOutputCurrents(time, qOutputCurrent)
% plotTCPForces(time, tcpForce);
plotTargetMoment(time, targetMoment);
% plotJointTemperatures(time, qTemperature);








function plotJointPositions(time, qs)
% plotJointPositions(time, qs)
%   time:     Nx1 time vector
%   qs:       Nx6 joint positions [q1 q2 ... q6] in radians

    figure;
    hold on;
    for k = 1:6
        plot(time, qs(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)',       'FontSize',12);
    ylabel('Joint Positions (rad)', 'FontSize',12);
    legend(arrayfun(@(k) sprintf('q_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('actual\_q','FontSize',14);
end


function plotJointVelocities(time, qds)
% plotJointVelocities(time, qds)
    figure;
    hold on;
    for k = 1:6
        plot(time, qds(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)',       'FontSize',12);
    ylabel('Joint Velocities (rad/s)', 'FontSize',12);
    legend(arrayfun(@(k) sprintf('q̇_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('actual\_qd','FontSize',14);
end


function plotTargetJointAccelerations(time, qdds)
% plotTargetJointAccelerations(time, qdds)
    figure;
    hold on;
    for k = 1:6
        plot(time, qdds(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)',       'FontSize',12);
    ylabel('Joint Accelerations (rad/s^2)', 'FontSize',12);
    legend(arrayfun(@(k) sprintf('q̇_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('target\_qdd','FontSize',14);
end


function plotTCPPose(time, tcpPose)
% plotTCPPose(time, tcpPose)
%   tcpPose: Nx6 [x y z Rx Ry Rz]

    labels = {'x(m)','y(m)','z(m)','Rx(rad)','Ry(rad)','Rz(rad)'};
    figure;
    hold on;
    for k = 1:6
        plot(time, tcpPose(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('TCP Pose','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('actual\_TCP\_pose w.r.t. base frame','FontSize',14);
end


function plotTCPSpeed(time, tcpSpeed)
% plotTCPSpeed(time, tcpSpeed)
%   tcpSpeed: Nx6 [vx vy vz wx wy wz]

    labels = {'v_x(m/s)','v_y(m/s)','v_z(m/s)','ω_x(rad/s)','ω_y(rad/s)','ω_z(rad/s)'};
    figure;
    hold on;
    for k = 1:6
        plot(time, tcpSpeed(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('TCP Speed','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('actual\_TCP\_speed w.r.t. base frame','FontSize',14);
end


function plotJointActualCurrents(time, qCurrent)
% plotJointActualCurrents(time, qCurrent)
%   qCurrent: Nx6 currents in mA

    figure;
    hold on;
    for k = 1:6
        plot(time, qCurrent(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('Joint Actual Currents (mA)','FontSize',12);
    legend(arrayfun(@(k) sprintf('I_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('actual\_current','FontSize',14);
end


function plotJointOutputCurrents(time, qCurrent)
% plotJointOutputCurrents(time, qCurrent)
%   qCurrent: Nx6 currents in mA

    figure;
    hold on;
    for k = 1:6
        plot(time, qCurrent(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('Joint Control Currents (mA)','FontSize',12);
    legend(arrayfun(@(k) sprintf('I_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('joint\_control\_output','FontSize',14);
end


function plotTCPForces(time, tcpForce)
% plotTCPForces(time, tcpForce)
%   tcpForce: Nx6 [Fx Fy Fz Tx Ty Tz]

    labels = {'F_x(N)','F_y(N)','F_z(N)','T_x(Nm)','T_y(Nm)','T_z(Nm)'};
    figure;
    hold on;
    for k = 1:6
        plot(time, tcpForce(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('TCP Forces / Torques','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('actual\_TCP\_force','FontSize',14);
end


function plotTargetMoment(time, targetMoment)
%   tcpForce: Nx6 [tau1 tau2 tau3 tau4 tau5 tau6]

    labels = {'tau_1','tau_2','tau_3','tau_4','tau_5','tau_6'};
    figure;
    hold on;
    for k = 1:6
        plot(time, targetMoment(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('Target Moments (Nm)','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('target\_moments','FontSize',14);
end

function plotJointTemperatures(time, qTemp)
%   tcpForce: Nx6 [temp1 temp2 temp3 temp4 temp5 temp6]

    labels = {'temp_1','temp_2','temp_3','temp_4','temp_5','temp_6'};
    figure;
    hold on;
    for k = 1:6
        plot(time, qTemp(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('Joint Temperature ({{\circ}}C)','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('joint\_temperatures','FontSize',14);
end


