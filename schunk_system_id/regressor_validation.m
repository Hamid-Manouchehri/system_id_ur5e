clc; 
clear; 
close all;

Y_fun = @Y_fun;

syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ... % Joint State
    m1 m2 m3 m4 m5 m6 ... % mi = m_li + m_m_i+1
    I_h1xx I_h1xy I_h1xz I_h1yy I_h1yz I_h1zz ... % Steiner Theorem : I_hixx = I_li + m_li*S(r_Ci,li)'*S(r_Ci,li) + m_i*(l_Ciy^2 + l_Ciz^2)
    I_h2xx I_h2xy I_h2xz I_h2yy I_h2yz I_h2zz ...
    I_h3xx I_h3xy I_h3xz I_h3yy I_h3yz I_h3zz ...
    I_h4xx I_h4xy I_h4xz I_h4yy I_h4yz I_h4zz ...
    I_h5xx I_h5xy I_h5xz I_h5yy I_h5yz I_h5zz ...
    I_h6xx I_h6xy I_h6xz I_h6yy I_h6yz I_h6zz ...
    m1_lC1x m1_lC1y m1_lC1z m2_lC2x m2_lC2y m2_lC2z m3_lC3x m3_lC3y m3_lC3z ... % page 262 _ eq 7.72 _ fig 7.13
    m4_lC4x m4_lC4y m4_lC4z m5_lC5x m5_lC5y m5_lC5z m6_lC6x m6_lC6y m6_lC6z...
    I_m1 I_m2 I_m3 I_m4 I_m5 I_m6 real;

% Dynamic parameters:
m = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

% center of mass [m]
m1_lC1 = [0.0, -0.02561, 0.00193];
m2_lC2 = [0.2125, 0.0, 0.11336];
m3_lC3 = [0.15, 0.0, 0.0265];
m4_lC4 = [0.0, -0.0018, 0.01634];
m5_lC5 = [0.0, 0.0018, 0.01634];
m6_lC6 = [0.0, 0.0, -0.001159];

% motor inertia
I_m = [6.6119 4.6147 7.9773 1.2249 1.1868 1.1981]*1e-5;

% link inertia [I_xx I_yy I_zz I_xy I_yz I_xz] [kgm^2]
I_h1 = [ 0.0067, 0.0064, 0.0067, 0.0, 0.0, 0.0];
I_h2 = [ 0.0149, 0.3564, 0.3553, 0.0, 0.0, 0.0];
I_h3 = [ 0.0025, 0.0551, 0.0546, 0.0, 0.0, 0.0034];
I_h4 = [ 0.0012, 0.0012, 0.0009, 0.0, 0.0, 0.0];
I_h5 = [ 0.0012, 0.0012, 0.009, 0.0, 0.0, 0.0];
I_h6 = [ 0.001, 0.001, 0.001, 0.0, 0.0, 0.0];

p = [m(1) m1_lC1(1) m1_lC1(2) m1_lC1(3) I_h1(1) I_h1(4) I_h1(6) I_h1(2) I_h1(5) I_h1(3) I_m(1) ...
     m(2) m2_lC2(1) m2_lC2(2) m2_lC2(3) I_h2(1) I_h2(4) I_h2(6) I_h2(2) I_h2(5) I_h2(3) I_m(2) ...
     m(3) m3_lC3(1) m3_lC3(2) m3_lC3(3) I_h3(1) I_h3(4) I_h3(6) I_h3(2) I_h3(5) I_h3(3) I_m(3) ...
     m(4) m4_lC4(1) m4_lC4(2) m4_lC4(3) I_h4(1) I_h4(4) I_h4(6) I_h4(2) I_h4(5) I_h4(3) I_m(4) ...
     m(5) m5_lC5(1) m5_lC5(2) m5_lC5(3) I_h5(1) I_h5(4) I_h5(6) I_h5(2) I_h5(5) I_h5(3) I_m(5) ...
     m(6) m6_lC6(1) m6_lC6(2) m6_lC6(3) I_h6(1) I_h6(4) I_h6(6) I_h6(2) I_h6(5) I_h6(3) I_m(6)]';



Ns=100;

% preallocation:
q   = zeros(6,Ns);
qd  = zeros(6,Ns);
qdd = zeros(6,Ns);
tau = zeros(6,Ns);

amp=0.5;
duration=5;
t = linspace(0, duration, Ns);

% default joint angles at home configuration: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
q(1,:) = 0.*ones(1,Ns); qd(1,:) = zeros(1,Ns); qdd(1,:) = zeros(1,Ns);
% q(2,:) = -pi/2*ones(1,Ns); qd(2,:) = zeros(1,Ns); qd6d(2,:) = zeros(1,Ns);
[q(2,:), qd(2,:), qdd(2,:)] = sinTrajectory(amp, duration, Ns, -pi/2, t);
% [q(3,:), qd(3,:), qdd(3,:)] = sinTrajectory(amp, duration, Ns, -pi/2, t);
q(3,:) = 0*ones(1,Ns); qd(3,:) = zeros(1,Ns); qdd(3,:) = zeros(1,Ns);
q(4,:) = -pi/2*ones(1,Ns); qd(4,:) = zeros(1,Ns); qdd(4,:) = zeros(1,Ns);
q(5,:) = 0.*ones(1,Ns); qd(5,:) = zeros(1,Ns); qdd(5,:) = zeros(1,Ns);
q(6,:) = 0.*ones(1,Ns); qd(6,:) = zeros(1,Ns); qdd(6,:) = zeros(1,Ns);

% Plot them all, with hold on
figure;
hold on;
plot(t, q(1,:), 'o');
plot(t, q(2,:), '+');
plot(t, q(3,:), '*');
plot(t, q(4,:), 'square');
plot(t, q(5,:), '-');
plot(t, q(6,:), '^');
hold off;

xlabel('time [s]');
ylabel('joint angles [rad]');
grid on;
legend("q1","q2","q3","q4","q5","q6","Location","best");

for i=1:numel(t)
    tau(:,i) = Y_fun(q(:,i), qd(:,i), qdd(:,i)) * p;
end

figure;
hold on;
plot(t, tau(1,:), 'o');
plot(t, tau(2,:), '+');
plot(t, tau(3,:), '*');
plot(t, tau(4,:), 'square');
plot(t, tau(5,:), '-');
plot(t, tau(6,:), '^');
hold off;
ylabel('torque [Nm]');
grid on;
legend(["tau1", "tau2", "tau3", "tau4", "tau5", "tau6", "location", "best"]);




%**************************************************************************
%************************** Function Definition ***************************
%**************************************************************************

function [q, qd, qdd] = sinTrajectory(amp, duration, Ns, offset, t)

  omega = 2*pi / duration;  % angular freq
  A = amp(:);  % ensure A is a column vector

  q   = zeros(numel(A), Ns);
  qd  = zeros(numel(A), Ns);
  qdd = zeros(numel(A), Ns);

  for i = 1:numel(A)
    q(i,:)   =   A(i) *  sin(omega * t) + offset;
    qd(i,:)  =   A(i) *  omega *  cos(omega * t);
    qdd(i,:) = - A(i) * (omega^2) * sin(omega * t);
  end
end
