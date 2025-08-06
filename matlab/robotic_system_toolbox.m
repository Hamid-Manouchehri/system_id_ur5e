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

ur5e_model = loadrobot("universalUR5e");
ur5e_model.DataFormat = 'row';
ur5e_model.Gravity = [0 0 -9.81];

% visualize the robot to verify
% figure;
% show(ur5e_model);
% title('UR5e Robot Model');

t_final = 10;
fs = 100;
t = linspace(0, t_final, t_final*fs)';

% create a smooth trajectory for 6 joints
q1_traj = deg2rad(90) * sin(2*pi*0.1*t);
q2_traj = 0. * t;
q3_traj = 0. * t;
q4_traj = 0. * t;
q5_traj = 0. * t;
q6_traj = 0. * t;
q_traj = [q1_traj q2_traj q3_traj q4_traj q5_traj q6_traj];

qd1_traj = 2*pi * 0.1 * deg2rad(90) * cos(2*pi*0.1*t);
qd2_traj = 0. * t;
qd3_traj = 0. * t;
qd4_traj = 0. * t;
qd5_traj = 0. * t;
qd6_traj = 0. * t;
qd_traj = [qd1_traj qd2_traj qd3_traj qd4_traj qd5_traj qd6_traj];

qdd1_traj = -4*pi^2 * 0.01 * deg2rad(90) * sin(2*pi*0.1*t);
qdd2_traj = 0. * t;
qdd3_traj = 0. * t;
qdd4_traj = 0. * t;
qdd5_traj = 0. * t;
qdd6_traj = 0. * t;
qdd_traj = [qdd1_traj qdd2_traj qdd3_traj qdd4_traj qdd5_traj qdd6_traj];

% inverse dynamics torques
tau = zeros(size(q_traj));
for i = 1:length(t)
    q   = q_traj(i,:);
    qd  = qd_traj(i,:);
    qdd = qdd_traj(i,:);
    
    tau(i,:) = inverseDynamics(ur5e_model, q, qd, qdd);
end

% Plot joint torques
figure;
for j = 1:6
    subplot(3,2,j);
    plot(t, tau(:,j),'LineWidth',1.2);
    grid on;
    title(sprintf('Joint %d Torque',j));
    xlabel('Time [s]');
    ylabel('\tau [Nm]');
end

% Visual animation of UR5e trajectory:
figure;
for k = 1:5:length(t)
    show(ur5e_model, q_traj(k,:), 'PreservePlot',false,'Frames','off');
    title(sprintf('UR5e Simulation (t=%.2fs)', t(k)));
    drawnow;
end
