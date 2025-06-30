%{
*******************************************************************************
Project: system-id ur5e
File: exciting_traj.py
Author: Hamid Manouchehri
Email: hmanouch@buffalo.edu
Date: June 22, 2025

Description:
Script for reading the calculated symbolic regressor and solving for the 
optimization problem to deriving the exciting trajectories.

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

% optimizeExcitingTraj.m
clc; 
clear; 
close all;

% 1) Load the symbolic regressor
tmp   = load('../data/mat/Y_sym_test.mat','Y_sym_test');
Y_sym = tmp.Y_sym_test;
Y_fun = @Y_fun_test;

% 3) Hyper‐parameters
nJ   = 6;     % DoF
N    = 4;     % harmonics
Tf   = 10;    % total duration [s]; TODO
Ns   = 50;    % time samples; TODO

qMin   = deg2rad([-360 -360 -360 -360 -360 -360])';
qMax   = deg2rad([ 360  360  360  360  360  360])';
qdLim  = deg2rad([180 180 180 180 180 180])';
qddLim = deg2rad([300 300 300 300 300 300])';

omega_f = 2*pi/Tf;
tGrid   = linspace(0,Tf,Ns);

% 4) Initial guess and bounds
nCoeff = 2*N*nJ;
ab0    = ones(nCoeff,1);

% here we just give loose infinite bounds; tighten as needed:
% LB = -Inf(nCoeff,1);
% UB =  Inf(nCoeff,1);
LB  = -[qMax; qdLim; qddLim];
UB  = -LB;
A   = []; b = [];
Aeq = []; beq = [];

% 6) fmincon options
opts = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'FiniteDifferenceType','forward', ...
    'UseParallel',true, ...
    'MaxFunctionEvaluations',3e4, ...
    'PlotFcn',{ @optimplotfval, ...            % objective vs iter
                @optimplotfirstorderopt, ...  % KKT optimality
                @optimplotstepsize, ...       % step size
                @optimplotconstrviolation });   % max constraint violation);

% 7) Build function handles
objFun = @(ab) objective(ab, Y_fun, tGrid, omega_f, nJ, N, qMin, qMax, size(Y_sym,2));
nonl   = [];  % or @nonlcon if you have constraints

% 8) Call fmincon, capturing all outputs
[abOpt, fval, exitflag, output, lambda, grad, hessian] = ...
    fmincon(objFun, ab0, A, b, Aeq, beq, LB, UB, nonl, opts);

% 9) Report solver diagnostics
% fprintf('\n=== fmincon results ===\n');
% fprintf('  exitflag = %d\n', exitflag);
% fprintf('  fval     = %g\n', fval);
% disp('  output = '),    disp(output)
% disp('  lambda = '),    disp(lambda)
% disp('  grad   = '),    disp(grad)
% disp('  hessian = '),   disp(hessian)

% fprintf('objective was called %d times\n', objective_counter);

% 10) Reconstruct and plot
[q,qd,qdd] = seriesFromCoeff(abOpt, tGrid, omega_f, nJ, N, qMin, qMax);
hold on;
figure
plot(tGrid, rad2deg(q).','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('q [deg]');
title('Optimized Trajectories');

% =========================================================================
% Local functions
% =========================================================================

function J = objective(ab, Y_fun, tGrid, omega_f, nJ, N, qMin, qMax, P)

    Ns = numel(tGrid);
    [qk, qdk, qddk] = seriesFromCoeff(ab, tGrid, omega_f, nJ, N, qMin, qMax);
    Ystk = zeros(nJ*Ns, P);
    for kk = 1:Ns
        idx = (kk-1)*nJ + (1:nJ);
        Ystk(idx,:) = Y_fun(qk(:,kk), qdk(:,kk), qddk(:,kk));
    end
    
    % compute cost
    s = svd(Ystk);
    condY = s(1)/s(end);
    sigma_min = s(end);
    J = condY + 1/sigma_min;
    
    if ~isfinite(J)
        % To avoid the following  error:
        % Objective function is undefined at initial point. Fmincon cannot
        % continue.
        J = 1e6;
    end
end

function [q, qd, qdd] = seriesFromCoeff(x, t, omega_f, nJ, N, qMin, qMax)
    
    Ns = numel(t);
    a  = reshape(x(1:nJ*N),      N, nJ);
    b  = reshape(x(nJ*N+1:end),  N, nJ);
    q0 = (qMax + qMin)/2;
    
    q   = zeros(nJ, Ns);
    qd  = zeros(nJ, Ns);
    qdd = zeros(nJ, Ns);
    
    for k = 1:Ns
        w   = omega_f * (1:N).';
        si  = sin(w * t(k));
        co  = cos(w * t(k));
        for i = 1:nJ
            ai = a(:,i);  bi = b(:,i);
            q(i,k)   = q0(i) + sum((ai./w).*si - (bi./w).*co);
            qd(i,k)  =          sum( ai.*co    +  bi.*si);
            qdd(i,k) =          sum(-ai.*w.*si +  bi.*w.*co);
        end
    end
end

