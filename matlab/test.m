% Example: Minimize f(x) = x1^2 + x2^2  subject to x1 + x2 = 1, x1 >= 0
% clc;
% f = @(x) x(1)^2 + x(2)^2;
% x0 = [0, 0];
% A = []; b = [];
% Aeq = []; beq = [];     % Equality: x1 + x2 = 1
% lb = []; ub = [];      % Lower bounds
% x_opt = fmincon(f, x0, A, b, Aeq, beq, lb, ub);
% disp(x_opt);


%% condition number and sv
clc; clear; close all;

lambda1 = 1;
lambda2 = 1;

% Objective function: takes a vector x = [x1, x2, x3]
obj_fcn = @(x) my_obj_fun(x, lambda1, lambda2);

x0 = [0 0 0];
x_max = [10 10 10];

A = []; b = [];
Aeq = []; beq = [];
ub = x_max;
lb = -x_max;

disp("computing fmincon");
opts = optimoptions('fmincon', 'Display','iter','MaxFunctionEvaluations',1e4);
q_opt = fmincon(obj_fcn, x0, A, b, Aeq, beq, lb, ub, [], opts);
disp(q_opt)

function cost = my_obj_fun(x, lambda1, lambda2)
    % Build A as a numeric matrix for current x
    x1 = x(1); x2 = x(2); x3 = x(3);
    A = [sin(x1) cos(x2)  1;
         2*x2    1+x3    x2];
    % Make sure A is full-rank (avoid inf/NaN cost)
    if rank(A) < min(size(A))
        cost = 1e6; % Penalize singular matrices
        return;
    end
    condition_num = cond(A);
    smallest_sv = min(svd(A));
    cost = lambda1 * condition_num + lambda2 / smallest_sv;
end

%%
clc; clear; %close all;

N = 4;
duration_s = 10
time_step = 0.1;
A = ones(N,1);
B = ones(N,1);
q0 = 0;
t = 1:time_step:duration_s;

q = fourier_all_joints(t, A, B, q0, N)


plot(t, q)

% function Q = fourier_all_joints(t, a, b, q0, N)
%     % Q: [length(t) x 6], each column is one joint's q(t)
%     if nargin < 5
%         q0 = zeros(6,1);
%     end
% 
%     omega_f = 2*pi/10;
%     T = t(:);              % column vector of times
% 
%     q   = zeros(length(T),1);
% 
%     for l = 1:N
% 
%         wfl = omega_f * l;
%         q   = q   + (a(l)/wfl)*sin(wfl*T) - (b(l)/wfl)*cos(wfl*T);
% 
%     end
%     Q   = q + q0;
% end




%%
clc; clear; close all

% Goal: minimize  f(x) = (x1-1)^2 + (x2+2)^2
% subject to:
%   1)  x1 + x2  ≤ 1        (linear inequality)
%   2)  x1 – x2  = 0.5      (linear equality)
%   3)  -2 ≤ x1 ≤  2        (bounds)
%      -inf ≤ x2 ≤  3


% ---------- objective --------------------------------------------
fun = @(x) (x(1)-1).^2 + (x(2)+2).^2;

% ---------- linear constraints -----------------------------------
A   = [1  1];      b   = 1;         % A*x ≤ b
Aeq = [1 -1];      beq = 0.5;       % Aeq*x = beq

% ---------- bounds ------------------------------------------------
lb = [-2  -Inf];
ub = [ 2   3];

% (no nonlinear constraints → pass [] )
nonlcon = [];

% ---------- initial guess -----------------------------------------
x0 = [0 0];

% ---------- solver options ----------------------------------------
opts = optimoptions('fmincon','Algorithm','sqp','Display','iter');

% ---------- run ---------------------------------------------------
[x_opt, fval, exitflag, output] = ...
          fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, opts);

% ---------- show result -------------------------------------------
fprintf('\nOptimal x  = [%.4f  %.4f]\n', x_opt);
fprintf('Optimal f  = %.4f\n', fval);

%%

clc; clear; close all;


% target curve
q_des = @(t) t.^2;

% objective: integrate (q(t)-q_des(t))^2 over [0,1]
fun = @(x) integral(@(t) (x(1)*t + x(2) - q_des(t)).^2, 0, 1);

% initial guess [a; b]
x0 = [0; 0];

% simple bounds on a and b
lb = [-5; -5];
ub = [ 5;  5];

% no linear or nonlinear constraints here
A = []; b = [];
Aeq = []; beq = [];
nonlcon = [];

opts = optimoptions('fmincon', 'Display','iter');
x_opt = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, opts);

fprintf('Optimal a = %.4f, b = %.4f\n', x_opt(1), x_opt(2));

% Plot result
t = linspace(0,1,100);
q_opt = x_opt(1)*t + x_opt(2);
figure;
plot(t, q_des(t), 'r--', 'LineWidth',1.5); hold on;
plot(t, q_opt,  'w-',  'LineWidth',1.5);
legend('t^2 (target)','a·t + b (opt)');
xlabel('t'); ylabel('q(t)');
grid on;


%%

clc; clear;

syms q_sym;

% problem data
N          = 4;
duration_s = 1;
dt         = 0.1;
t          = 0:dt:duration_s;
q0         = 0;
omega_f    = pi / 5;

% initial guess (e.g. all ones)
A0 = ones(N,1);
B0 = ones(N,1);
x0 = [A0; B0];

% bounds: 1 < a_i < 2,   3 < b_i < 5
lb = [1*ones(N,1) ; 3*ones(N,1)];
ub = [2*ones(N,1) ; 5*ones(N,1)];

Y_sym = [ q_sym,        sin(q_sym); 
      cos(q_sym),      q_sym  ];

% objective: squared‐energy of q(t)
% fun = @(x) cost_quad(x, t, q0, N, dt);
% obj_t = @(x) cond_cost(x, t, q0, N, Y, q_sym, omega_f);

% opts = optimoptions('fmincon', 'Display','iter', 'Algorithm','sqp');
opts = optimoptions('fmincon', 'Algorithm', 'sqp');

% Preallocate storage for the optimal Fourier‐coeffs at each t_k
X_opt = zeros(2*N, numel(t));

for k = 1:numel(t)
    tk = t(k);
    % build a handle that only cares about this single time tk
    obj_at_tk = @(x) cond_at_time(x, tk, q0, N, Y_sym, q_sym, omega_f);

    % solve the small 2N-var problem for this tk
    X_opt(:,k) = fmincon(obj_at_tk, x0, [],[],[],[], lb, ub, [], opts);
    disp(["time:", tk, "fmincon:", X_opt(:, k)'])
end





% A_opt = x_opt(1:N);
% B_opt = x_opt(N+1:end);

% fprintf('A_opt = [%s]\n', sprintf(' %.3f',A_opt));
% fprintf('B_opt = [%s]\n', sprintf(' %.3f',B_opt));

% plot result
% q_opt = fourier_all_joints(t, A_opt, B_opt, q0, N);
% figure; plot(t,q_opt,'LineWidth',1.5);
% xlabel('time (s)'), ylabel('q(t)'), grid on


function J = cond_at_time(x, tk, q0, N, Y_sym, q_sym, omega_f)
    % x = [A(1:N); B(1:N)]
    % tk = scalar time
    % compute q1(tk) from the Fourier coefficients
    a = x(1:N);
    b = x(N+1:end);
    q1 = q0;
    for l = 1:N
        wfl = omega_f * l;
        q1 = q1 + (a(l)/wfl)*sin(wfl*tk) - (b(l)/wfl)*cos(wfl*tk);
    end

    % substitute into the symbolic regressor and get cond
    Y = double( subs(Y_sym, q_sym, q1) );
    s = svd(Y);
    
    % compute condition number and min singular value:
    condY = s(1)/s(end);
    minSignVal = s(end);
    J = condY + 1 / minSignVal;
end

%% test fourier-series function
clc; clear; close all;

nJ   = 1;
N    = 4;        % harmonics per joint
Tf   = 10;       % TODO; fundamental period / duration [s]
Ns   = 50;      % TODO; number of time samples

omega_f = 2*pi/Tf;
tGrid   = linspace(0,Tf,Ns);

[q, qd, qdd] = seriesFromCoeff(x, tGrid, omega_f, nJ, N);

figure;
plot(tGrid, rad2deg(traj.q).','LineWidth',1.4);
grid on;
xlabel('time [s]'); ylabel('q [deg]');
title('Optimised Joint Motions');

function [q, qd, qdd] = seriesFromCoeff(x, t, omega_f, nJ, N)
    Ns = numel(t);
    a  = reshape(x(1:nJ*N),      N, nJ);
    b  = reshape(x(nJ*N+1:end),  N, nJ);
    q0 = 0;

    q   = zeros(nJ, Ns);
    qd  = zeros(nJ, Ns);
    qdd = zeros(nJ, Ns);

    for k = 1:Ns
        w  = omega_f * (1:N).';
        si = sin(w * t(k));
        co = cos(w * t(k));
        for i = 1:nJ
            ai = a(:,i);  bi = b(:,i);
            q(i,k)   = q0(i) + sum((ai./w).*si - (bi./w).*co);
            qd(i,k)  =          sum( ai.*co    +  bi.*si);
            qdd(i,k) =          sum(-ai.*w.*si +  bi.*w.*co);
        end
    end
end