%{
*******************************************************************************
Project: system-id ur5e
File: joint_traj_gen.m
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: July 25, 2025

Description:
Script for generating trajectory for ur5e.

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

filename = 'ur5e_smooth_elbow_ninty_deg.csv'; % TODO: any name you like, TODO
folder   = '../data/traj/';
if ~exist(folder, 'dir')
    mkdir(folder);
end
csvPath = fullfile(folder, filename);

T = 20;                % total duration, TODO
Ns = 1000;
nJ = 6;
t = linspace(0, T, Ns);

q_offset = [0, -pi/2, 0, -pi/2, 0., 0.];
A = [60, 30., -30., -30., 30., 90.];  % Amplitudes (deg), TODO
A = deg2rad(A);
freqs = [0.05, 0.05, 0.05, 0.05, .05, 0.05];  % Hz, TODO


% smooth ramp (Hann window) for start/stop
rampTime = 2.5;          % duration of ramp up/down in sec, TODO
ramp = ones(size(t));
idxUp = t < rampTime;
ramp(idxUp) = 0.5 * (1 - cos(pi * t(idxUp) / rampTime));
idxDown = t > (T - rampTime);
ramp(idxDown) = 0.5 * (1 - cos(pi * (T - t(idxDown)) / rampTime));
% ramp = 1;  % uncomment for no smoothening.

% final trajectory
q = zeros(nJ, Ns);
% q(1,:) = A(1) * sin(2*pi*freqs(1)*t) .* ramp + q_offset(1);
% q(2,:) = A(2) * sin(2*pi*freqs(2)*t) .* ramp + q_offset(2);
% q(3,:) = A(3) * sin(2*pi*freqs(3)*t) .* ramp + q_offset(3);
% q(4,:) = A(4) * sin(2*pi*freqs(4)*t) .* ramp + q_offset(4);
% q(5,:) = A(5) * sin(2*pi*freqs(5)*t) .* ramp + q_offset(5);
% q(6,:) = A(6) * sin(2*pi*freqs(6)*t) .* ramp + q_offset(6);

q(1,:) = 0. * t;
q(2,:) = -pi/2 * ones(Ns, 1);
q(3,:) = deg2rad(90) * sin(2*pi*freqs(3)*t) .* ramp;
q(4,:) = -pi/2 * ones(Ns, 1);
q(5,:) = 0. * t;
q(6,:) = 0. * t;

figure;
plot(t, q');   % plot each joint (degrees for readability)
grid on;
xlabel('Time [s]');
ylabel('Joint Angle [rad]');
title('UR5e Joint Trajectories');
legend('q_1','q_2','q_3','q_4','q_5','q_6','Location','best');

traj = [t', q'];

% 3) Write header + data
hdr = {'time','q1','q2','q3','q4','q5','q6'};

fid = fopen(csvPath,'w');
fprintf(fid, '%s,', hdr{1:end-1});
fprintf(fid, '%s\n', hdr{end});   % newline after last header
fclose(fid);

% Append numeric data (comma-separated, 6 decimals)
writematrix(traj, csvPath, 'WriteMode','append');

fprintf('Saved trajectory to:  %s\n', csvPath);