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

fullfile = fullfile('/home/hamid/projects/system_id_ur5e/data/logs/', 'ur5e_data_test.csv');

% 2) Read into a table (handles headers automatically):
T = readtable(fullfile);

% 3) Convert to array for easy indexing:
data = table2array(T);

% 4) Assume column 1 is time, the rest are signals:
time      = data(:,1);
qs        = data(:,2:7);
qds       = data(:,8:13);
tcpPose   = data(:,14:19);
tcpSpeed  = data(:,20:25);
qCurrent  = data(:,26:31);
tcpForce  = data(:,32:37);

groups = {qs, qds, tcpPose, tcpSpeed, qCurrent, tcpForce};
groupNames = {'Joint Positions (rad)', ...
              'Joint Velocities (rad/s)', ...
              'TCP Pose [x y z Rx Ry Rz]', ...
              'TCP Speed [vx vy vz wx wy wz]', ...
              'Joint Currents (mA)', ...
              'TCP Forces/Torques (N/Nm)'};

for i = 1:numel(groups)
    figure;
    hold on;
    S = groups{i};
    for k = 1:size(S,2)
        plot(time, S(:,k), 'LineWidth',1.5);
    end
    hold off;
    xlabel('Time (s)',       'FontSize',12);
    ylabel(groupNames{i},   'FontSize',12);
    legend_str = arrayfun(@(k) sprintf('chan %d',k), 1:size(S,2), 'UniformOutput',false);
    legend(legend_str, 'Location','best');
    grid on;
    title(groupNames{i}, 'FontSize',14);
end

