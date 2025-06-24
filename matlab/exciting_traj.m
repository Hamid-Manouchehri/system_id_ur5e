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
clc; %clear; close all;

disp("loading regressor .mat file...");
load("../data/mat/Y_sym.mat");

[xOpt, traj, Ybig] = designExcitingTrajectory(Y_sym);

% quick visual check
figure; plot(traj.t, rad2deg(traj.q).');  grid on
xlabel('time [s]'); ylabel('q [deg]'); title('Optimised joint motion');
fprintf('cond(Ybig) = %.3e\n', cond(Ybig));



function [xOpt, traj, Ybig] = designExcitingTrajectory(Y_sym)
% -------------------------------------------------------------------------
%  Exciting-trajectory design for dynamic-parameter identification
%  (SCHUNK Powerball example – easily adapted to any n-DoF chain).
%
%  INPUT
%    Y_sym  – symbolic regressor  (n×P)  from buildRegressorSymbolic()
%
%  OUTPUT
%    xOpt   – optimal Fourier-coefficient vector  [a(:); b(:)]
%    traj   – struct with fields .t, .q, .qd, .qdd  (n×Ns each)
%    Ybig   – stacked regressor along the trajectory (n*Ns × P)
% -------------------------------------------------------------------------

% -------- USER SECTION ---------------------------------------------------
nJ  = 6;             % DoF
N   = 4;             % # harmonics
Tf  = 2;            % fundamental period  (ω_f = 2π/Tf)
Ns  = 5;           % # time samples  (≈ optimisation grid)

qMin   = deg2rad([-170 -120 -170 -120 -170 -120]);  %  << joint limits
qMax   = deg2rad([ 170  120  170  120  170  120]);
qdLim  = deg2rad([ 120  120  120  180  180  180]);  %  velocity limits
qddLim = deg2rad([ 300  300  300  300  300  300]);  %  acceleration limits
% -------------------------------------------------------------------------

omega_f = 2*pi/Tf;
tGrid   = linspace(0,Tf,Ns);

% ---- convert symbolic regressor to fast numeric function for Y ---------- 
disp("changing Y_sym to Y_fun...");
qSym   = sym('q',[6,1]);
dqSym  = sym('dq',[6,1]);
ddqSym = sym('ddq',[6,1]);
Yfun = matlabFunction(Y_sym, 'Vars', {qSym, dqSym, ddqSym});

% ---- coefficient initial guess & conservative bounds --------------------
nCoeff = 2*N*nJ;
x0     = zeros(nCoeff,1);                      % start from rest

ampPos = (qMax-qMin)/2;                        % mid-range amplitude
UB     =  repmat(ampPos(:).*omega_f,2*N,1);    % position-based cap
LB     = -UB;

%--- parallel pool (optional) -------------------------------
% only do this if you really want parallel finite‐diff
if isempty(gcp('nocreate'))
    try
      parpool;   % start with default profile
    catch
      warning('Couldn''t start parallel pool: continuing in serial.');
    end
end
% ---- fmincon setup ------------------------------------------------------
opts = optimoptions('fmincon', ...
            'Algorithm',       'sqp', ...
            'Display',         'iter', ...
            'UseParallel',     true, ...  % now only if pool exists
            'MaxFunctionEvaluations', 3e4);

% 2) debug at x0, before fmincon:
J0 = objective(x0);
[c0,ceq0] = nonlcon(x0);
fprintf('J0 = %g; c0 min = %g; ceq0 min = %g\n', J0, min(c0), min(ceq0));

disp("computing fmincon...")
[xOpt,~,exitflag,output] = fmincon(@objective,x0,[],[],[],[],LB,UB,[],opts);

fprintf('\nExitflag %d  –  %s\n',exitflag,output.message);

% ---- reconstruct optimised trajectory & full regressor -----------------
[q,qd,qdd] = seriesFromCoeff(xOpt,tGrid,omega_f,nJ,N,qMin,qMax);
traj = struct('t',tGrid,'q',q,'qd',qd,'qdd',qdd);

Ybig = zeros(nJ*Ns,size(Y_sym,2));
for k = 1:Ns
    idx = (k-1)*nJ+(1:nJ);
    Ybig(idx,:) = Yfun(q(:,k),qd(:,k),qdd(:,k));
end
% -------------------------------------------------------------------------

% ================= nested objective & constraints =======================

    function J = objective(x)
        % build trajectory
        [qk, qdk, qddk] = seriesFromCoeff(x, tGrid, omega_f, nJ, N, qMin, qMax);

        % stack the regressor
        Ystk = zeros(nJ*Ns, size(Y_sym,2));
        for kk = 1:Ns
            idx = (kk-1)*nJ + (1:nJ);
            M = Yfun(qk(:,kk), qdk(:,kk), qddk(:,kk));
            Ystk(idx,:) = M;
        end

        % compute condition number, guard against singularity
        J = cond(Ystk);
        if ~isfinite(J)    % catches Inf or NaN
            J = 1e6;       % a large finite penalty
        end
    end


    % function [c,ceq] = nonlcon(x)
    %     [qk,qdk,qddk] = seriesFromCoeff(x, tGrid, omega_f, nJ, N, qMin, qMax);
    % 
    %     % enforce the box‐limits
    %     c = [ reshape(qk-qMax(:),[],1);
    %           reshape(qMin(:)-qk,[],1);
    %           reshape(qdk-qdLim(:),[],1);
    %           reshape(-qdk-qdLim(:),[],1);
    %           reshape(qddk-qddLim(:),[],1);
    %           reshape(-qddk-qddLim(:),[],1) ];
    % 
    %     % if anything went NaN, make it obviously infeasible
    %     if any(isnan(qk(:))) || any(isnan(qdk(:))) || any(isnan(qddk(:)))
    %         c = 1e6*ones(size(c));
    %     end
    % 
    %     ceq = [];  % no equality constraints
    % end

end
% ================ END OF MAIN FUNCTION ===================================


% ========================================================================
%  Local utility: Fourier-series → q,qd,qdd  (kept as sub-function)
% ========================================================================
function [q,qd,qdd] = seriesFromCoeff(x,t,omega_f,nJ,N,qMin,qMax)
    disp("inside seriesFromCoeff");
    Ns  = numel(t);
    a   = reshape(x(1:nJ*N),      N,nJ);   % N×nJ
    b   = reshape(x(nJ*N+1:end),  N,nJ);
    
    q0  = (qMax+qMin)/2;                    % centre of each joint range
    q   = zeros(nJ,Ns);
    qd  = zeros(nJ,Ns);
    qdd = zeros(nJ,Ns);
    
    for k = 1:Ns
        w = omega_f*(1:N).';                    % column vector
        si = sin(w*t(k));
        co = cos(w*t(k));
        for i = 1:nJ
            ai = a(:,i);  bi = b(:,i);
            q(i,k)   = q0(i) + sum( (ai./w).*si - (bi./w).*co );
            qd(i,k)  =          sum(  ai.*co   +  bi.*si   );
            qdd(i,k) =          sum( -ai.*w.*si + bi.*w.*co );
        end
    end
    disp("jump out of seriesFromCoeff");
end


