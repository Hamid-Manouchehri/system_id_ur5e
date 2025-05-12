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

csv_file_name = 'ur5e_data_test.csv';  % TODO
csv_dir = '/home/hamid/projects/system_id_ur5e/data/logs/';

fullfile = fullfile(csv_dir, csv_file_name);

% 2) Read into a table (handles headers automatically):
T = readtable(fullfile);

% 3) Convert to array for easy indexing:
data = table2array(T);

time      = data(:,1);
qs        = data(:,2:7);
qds       = data(:,8:13);
tcpPose   = data(:,14:19);
tcpSpeed  = data(:,20:25);
qCurrent  = data(:,26:31);
tcpForce  = data(:,32:37);






plotJointPositions(time, qs);
plotJointVelocities(time, qds);
plotTCPPose(time, tcpPose);
plotTCPSpeed(time, tcpSpeed);
plotJointCurrents(time, qCurrent);
plotTCPForces(time, tcpForce);








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
    title('Joint Positions','FontSize',14);
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
    title('Joint Velocities','FontSize',14);
end


function plotTCPPose(time, tcpPose)
% plotTCPPose(time, tcpPose)
%   tcpPose: Nx6 [x y z Rx Ry Rz]

    labels = {'x','y','z','Rx','Ry','Rz'};
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
    title('TCP Pose','FontSize',14);
end


function plotTCPSpeed(time, tcpSpeed)
% plotTCPSpeed(time, tcpSpeed)
%   tcpSpeed: Nx6 [vx vy vz wx wy wz]

    labels = {'v_x','v_y','v_z','ω_x','ω_y','ω_z'};
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
    title('TCP Speed','FontSize',14);
end


function plotJointCurrents(time, qCurrent)
% plotJointCurrents(time, qCurrent)
%   qCurrent: Nx6 currents in mA

    figure;
    hold on;
    for k = 1:6
        plot(time, qCurrent(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('Joint Currents (mA)','FontSize',12);
    legend(arrayfun(@(k) sprintf('I_%d',k), 1:6, 'Uni',false),...
           'Location','best');
    grid on;
    title('Joint Currents','FontSize',14);
end


function plotTCPForces(time, tcpForce)
% plotTCPForces(time, tcpForce)
%   tcpForce: Nx6 [Fx Fy Fz Tx Ty Tz]

    labels = {'F_x','F_y','F_z','T_x','T_y','T_z'};
    figure;
    hold on;
    for k = 1:6
        plot(time, tcpForce(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)','FontSize',12);
    ylabel('TCP Forces/Torques','FontSize',12);
    legend(labels,'Location','best');
    grid on;
    title('TCP Forces and Torques','FontSize',14);
end

