%%% Recalculating PowerBall motion equation using linearity property of the
%%% equation
clc
clear all
% Kinetic energy and potential energy are calculated according to page 262
% Siciliano textbook

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


p = [m1 m1_lC1x m1_lC1y m1_lC1z I_h1xx I_h1xy I_h1xz I_h1yy I_h1yz I_h1zz I_m1 ...
     m2 m2_lC2x m2_lC2y m2_lC2z I_h2xx I_h2xy I_h2xz I_h2yy I_h2yz I_h2zz I_m2 ...
     m3 m3_lC3x m3_lC3y m3_lC3z I_h3xx I_h3xy I_h3xz I_h3yy I_h3yz I_h3zz I_m3 ...
     m4 m4_lC4x m4_lC4y m4_lC4z I_h4xx I_h4xy I_h4xz I_h4yy I_h4yz I_h4zz I_m4 ...
     m5 m5_lC5x m5_lC5y m5_lC5z I_h5xx I_h5xy I_h5xz I_h5yy I_h5yz I_h5zz I_m5 ...
     m6 m6_lC6x m6_lC6y m6_lC6z I_h6xx I_h6xy I_h6xz I_h6yy I_h6yz I_h6zz I_m6]';
 
%%%%%% Simplification %%%%%%%%
m1_lC1x = 0;
m1_lC1z = 0;
m2_lC2y = 0;
m3_lC3x = 0;
m4_lC4x = 0;
m5_lC5x = 0;
m6_lC6x = 0;
m6_lC6y = 0;
I_h1xy = 0; I_h1xz = 0; I_h1yz = 0;
I_h2xy = 0; I_h2yz = 0;
I_h3xy = 0; I_h3xz = 0;
I_h4xy = 0; I_h4xz = 0;
I_h5xy = 0; I_h5xz = 0;
I_h6xy = 0; I_h6xz = 0; I_h6yz = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m1_rC1 = [0. -0.02561 0.00193]';
m2_rC2 = [0.2125 0. 0.11336]';
m3_rC3 = [0.15 0. 0.0265]';
m4_rC4 = [0. -0.0018 0.01634]';
m5_rC5 = [0. 0.0018 0.01634]';
m6_rC6 = [0. 0. -0.001159]';

I_1_h = [I_h1xx -I_h1xy -I_h1xz;
    -I_h1xy I_h1yy -I_h1yz;
    -I_h1xz -I_h1yz I_h1zz];

I_2_h = [I_h2xx -I_h2xy -I_h2xz;
    -I_h2xy I_h2yy -I_h2yz;
    -I_h2xz -I_h2yz I_h2zz];

I_3_h = [I_h3xx -I_h3xy -I_h3xz;
    -I_h3xy I_h3yy -I_h3yz;
    -I_h3xz -I_h3yz I_h3zz];

I_4_h = [I_h4xx -I_h4xy -I_h4xz;
    -I_h4xy I_h4yy -I_h4yz;
    -I_h4xz -I_h4yz I_h4zz];

I_5_h = [I_h5xx -I_h5xy -I_h5xz;
    -I_h5xy I_h5yy -I_h5yz;
    -I_h5xz -I_h5yz I_h5zz];

I_6_h = [I_h6xx -I_h6xy -I_h6xz;
    -I_h6xy I_h6yy -I_h6yz;
    -I_h6xz -I_h6yz I_h6zz];

%%%%%%%%%%%%%%%%%% SCHUNK Arm Kinematics & properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;
k_r1 = 1;
k_r2 = 1;
k_r3 = 1;
k_r4 = 1;
k_r5 = 1;
k_r6 = 1;

T_1 = [ cos(q1), 0,  sin(q1), 0;
  sin(q1), 0, -cos(q1), 0;
        0, 1,        0, 0;
        0, 0,        0, 1];

T_2 = [ -cos(q1)*sin(q2), cos(q1)*cos(q2), -sin(q1), -(7*cos(q1)*sin(q2))/20;
  -sin(q1)*sin(q2), cos(q2)*sin(q1),  cos(q1), -(7*sin(q1)*sin(q2))/20;
           cos(q2),         sin(q2),        0,          (7*cos(q2))/20;
                 0,               0,        0,                       1];

T_3 = [ cos(q2 - q3)*cos(q1), -sin(q1), -sin(q2 - q3)*cos(q1), -(7*cos(q1)*sin(q2))/20;
  cos(q2 - q3)*sin(q1),  cos(q1), -sin(q2 - q3)*sin(q1), -(7*sin(q1)*sin(q2))/20;
          sin(q2 - q3),        0,          cos(q2 - q3),          (7*cos(q2))/20;
                     0,        0,                     0,                       1];

T_4 = [ cos(q1)*cos(q2)*cos(q3)*cos(q4) - sin(q1)*sin(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3), sin(q2 - q3)*cos(q1), - cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4), -(cos(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
  cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) + cos(q4)*sin(q1)*sin(q2)*sin(q3), sin(q2 - q3)*sin(q1),   cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) - sin(q1)*sin(q2)*sin(q3)*sin(q4), -(sin(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
                                                                 sin(q2 - q3)*cos(q4),        -cos(q2 - q3),                                                                 -sin(q2 - q3)*sin(q4),        (61*cos(q2 - q3))/200 + (7*cos(q2))/20;
                                                                                    0,                    0,                                                                                     0,                                             1];
 
T_5 = [ cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3), - cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), -(cos(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
  cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3),   cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) - sin(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), -(sin(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
                                                            cos(q3)*cos(q4)*cos(q5)*sin(q2) - cos(q2)*cos(q3)*sin(q5) - cos(q2)*cos(q4)*cos(q5)*sin(q3) - sin(q2)*sin(q3)*sin(q5),                                                                 -sin(q2 - q3)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5),        (61*cos(q2 - q3))/200 + (7*cos(q2))/20;
                                                                                                                                                                                0,                                                                                     0,                                                                                                                                                                               0,                                             1];
 
T_6 = [ cos(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), -(cos(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
  cos(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) - sin(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), -(sin(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200;
                                                                                                                           - cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*sin(q4)*sin(q6),                                                                                                                              sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*cos(q6)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5),        (61*cos(q2 - q3))/200 + (7*cos(q2))/20;
                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                               0,                                             1];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R_1 = T_1(1:3,1:3);
R_2 = T_2(1:3,1:3);
R_3 = T_3(1:3,1:3);
R_4 = T_4(1:3,1:3);
R_5 = T_5(1:3,1:3);
R_6 = T_6(1:3,1:3);

z0 = [0 0 1]';
z1 = T_1(1:3,3);
z2 = T_2(1:3,3);
z3 = T_3(1:3,3);
z4 = T_4(1:3,3);
z5 = T_5(1:3,3);
z6 = T_6(1:3,3);

p0 = [0 0 0]';
p1 = T_1(1:3,4);
p2 = T_2(1:3,4);
p3 = T_3(1:3,4);
p4 = T_4(1:3,4);
p5 = T_5(1:3,4);
p6 = T_6(1:3,4);

J_p1_P1 = cross(z0,(p1-p0));
J_p2_P1 = cross(z0,(p2-p0));
J_p3_P1 = cross(z0,(p3-p0));
J_p4_P1 = cross(z0,(p4-p0));
J_p5_P1 = cross(z0,(p5-p0));
J_p6_P1 = cross(z0,(p6-p0));

J_p2_P2 = cross(z1,(p2-p1));
J_p3_P2 = cross(z1,(p3-p1));
J_p4_P2 = cross(z1,(p4-p1));
J_p5_P2 = cross(z1,(p5-p1));
J_p6_P2 = cross(z1,(p6-p1));

J_p3_P3 = cross(z2,(p3-p2));
J_p4_P3 = cross(z2,(p4-p2));
J_p5_P3 = cross(z2,(p5-p2));
J_p6_P3 = cross(z2,(p6-p2));

J_p4_P4 = cross(z3,(p4-p3));
J_p5_P4 = cross(z3,(p5-p3));
J_p6_P4 = cross(z3,(p6-p3));

J_p5_P5 = cross(z4,(p5-p4));
J_p6_P5 = cross(z4,(p6-p4));

J_p6_P6 = cross(z5,(p6-p5));

J_p1 = [J_p1_P1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p2 = [J_p2_P1    J_p2_P2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p3 = [J_p3_P1    J_p3_P2    J_p3_P3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_p4 = [J_p4_P1    J_p4_P2    J_p4_P3    J_p4_P4 zeros(3,1) zeros(3,1)];
J_p5 = [J_p5_P1    J_p5_P2    J_p5_P3    J_p5_P4    J_p5_P5 zeros(3,1)];
J_p6 = [J_p6_P1    J_p6_P2    J_p6_P3    J_p6_P4    J_p6_P5    J_p6_P6];

q = [q1 q2 q3 q4 q5 q6]';
dq = [dq1 dq2 dq3 dq4 dq5 dq6]';
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6]';

dp_1 = J_p1*dq;
dp_2 = J_p2*dq;
dp_3 = J_p3*dq;
dp_4 = J_p4*dq;
dp_5 = J_p5*dq;
dp_6 = J_p6*dq;

J_p1_O1 = z0;
J_p2_O1 = z0;
J_p3_O1 = z0;
J_p4_O1 = z0;
J_p5_O1 = z0;
J_p6_O1 = z0;

J_p2_O2 = z1;
J_p3_O2 = z1;
J_p4_O2 = z1;
J_p5_O2 = z1;
J_p6_O2 = z1;

J_p3_O3 = z2;
J_p4_O3 = z2;
J_p5_O3 = z2;
J_p6_O3 = z2;

J_p4_O4 = z3;
J_p5_O4 = z3;
J_p6_O4 = z3;

J_p5_O5 = z4;
J_p6_O5 = z4;

J_p6_O6 = z5;

J_p1_O = [J_p1_O1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p2_O = [J_p2_O1    J_p2_O2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_p3_O = [J_p3_O1    J_p3_O2    J_p3_O3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_p4_O = [J_p4_O1    J_p4_O2    J_p4_O3    J_p4_O4 zeros(3,1) zeros(3,1)];
J_p5_O = [J_p5_O1    J_p5_O2    J_p5_O3    J_p5_O4    J_p5_O5 zeros(3,1)];
J_p6_O = [J_p6_O1    J_p6_O2    J_p6_O3    J_p6_O4    J_p6_O5    J_p6_O6];

w_1 = J_p1_O*dq;
w_2 = J_p2_O*dq;
w_3 = J_p3_O*dq;
w_4 = J_p4_O*dq;
w_5 = J_p5_O*dq;
w_6 = J_p6_O*dq;


S_w1 = [0 -w_1(3) w_1(2);
    w_1(3) 0 -w_1(1);
    -w_1(2) w_1(1) 0];

S_w2 = [0 -w_2(3) w_2(2);
    w_2(3) 0 -w_2(1);
    -w_2(2) w_2(1) 0];

S_w3 = [0 -w_3(3) w_3(2);
    w_3(3) 0 -w_3(1);
    -w_3(2) w_3(1) 0];

S_w4 = [0 -w_4(3) w_4(2);
    w_4(3) 0 -w_4(1);
    -w_4(2) w_4(1) 0];

S_w5 = [0 -w_5(3) w_5(2);
    w_5(3) 0 -w_5(1);
    -w_5(2) w_5(1) 0];

S_w6 = [0 -w_6(3) w_6(2);
    w_6(3) 0 -w_6(1);
    -w_6(2) w_6(1) 0];

disp('Kinetic Energy 0');
T0 = 1/2*k_r1^2*dq1^2*I_m1; %T0 = simple(T0);
disp('Kinetic Energy 1');
T1 = 1/2*m1*(dp_1'*dp_1) + dp_1'*S_w1*m1_rC1 + 1/2*w_1'*R_1*I_1_h*R_1'*w_1 + k_r2*dq2*I_m2*z2'*w_1 + 1/2*k_r2^2*dq2^2*I_m2; %T1 = simple(T1);
disp('Kinetic Energy 2');
T2 = 1/2*m2*(dp_2'*dp_2) + dp_2'*S_w2*m2_rC2 + 1/2*w_2'*R_2*I_2_h*R_2'*w_2 + k_r3*dq3*I_m3*z3'*w_2 + 1/2*k_r3^2*dq3^2*I_m3; %T2 = simple(T2);
disp('Kinetic Energy 3');
T3 = 1/2*m3*(dp_3'*dp_3) + dp_3'*S_w3*m3_rC3 + 1/2*w_3'*R_3*I_3_h*R_3'*w_3 + k_r4*dq4*I_m4*z4'*w_3 + 1/2*k_r4^2*dq4^2*I_m4; %T3 = simple(T3);
disp('Kinetic Energy 4');
T4 = 1/2*m4*(dp_4'*dp_4) + dp_4'*S_w4*m4_rC4 + 1/2*w_4'*R_4*I_4_h*R_4'*w_4 + k_r5*dq5*I_m5*z5'*w_4 + 1/2*k_r5^2*dq5^2*I_m5; %T4 = simple(T4);
disp('Kinetic Energy 5');
T5 = 1/2*m5*(dp_5'*dp_5) + dp_5'*S_w5*m5_rC5 + 1/2*w_5'*R_5*I_5_h*R_5'*w_5 + k_r6*dq6*I_m6*z6'*w_5 + 1/2*k_r6^2*dq6^2*I_m6;
disp('Kinetic Energy 6');
T6 = 1/2*m6*(dp_6'*dp_6) + dp_6'*S_w6*m6_rC6 + 1/2*w_6'*R_6*I_6_h*R_6'*w_6;

disp('Kinetic Energy is calculating');
T = T0 + T1 + T2 + T3 + T4 + T5 + T6;
disp('Kinetic Energy is calculated');

g_0 = [0 0 -g]';
U1 = -g_0'*(m1*p1 + m1_rC1);
U2 = -g_0'*(m2*p2 + m2_rC2);
U3 = -g_0'*(m3*p3 + m3_rC3);
U4 = -g_0'*(m4*p4 + m4_rC4);
U5 = -g_0'*(m5*p5 + m5_rC5);
U6 = -g_0'*(m6*p6 + m6_rC6);

U = U1 + U2 + U3 + U4 + U5 + U6;
disp('Potential Energy is calculated');

state = [q' dq']';
dstate = [dq' ddq']';

L = T-U;

disp('differating jacobian');
L_dq = jacobian(L,dq)';
L_dq_t = jacobian(L_dq,state)*dstate;
L_q = jacobian(L,q)';
Eq_l = L_dq_t - L_q;
disp('motion equation is calculated');
save Eq_l Eq_l

Y_simp = jacobian(Eq_l,p);
save Y_sym Y_sym
save p p

disp("saving Y_sym as a function...");
qSym   = sym('q',[6,1]);
dqSym  = sym('dq',[6,1]);
ddqSym = sym('ddq',[6,1]);
matlabFunction(Y_simp, 'File','Y_fun','Vars',{qSym,dqSym,ddqSym});
Y_fun = matlabFunction(Y_sym);








