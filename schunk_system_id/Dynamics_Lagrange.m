%%% Calculating PowerBall dynamics using lagrange method %%%
clc
clear all

syms g q1 q2 q3 q4 q5 q5 q6 dq1 dq2 dq3 dq4 dq5 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ...
    m_l1 m_l2 m_l3 m_l4 m_l5 m_l6 m_m1 m_m2 m_m3 m_m4 m_m5 m_m6 ...
    I_l1xx I_l1xy I_l1xz I_l1yy I_l1yz I_l1zz ...
    I_l2xx I_l2xy I_l2xz I_l2yy I_l2yz I_l2zz ...
    I_l3xx I_l3xy I_l3xz I_l3yy I_l3yz I_l3zz ...
    I_l4xx I_l4xy I_l4xz I_l4yy I_l4yz I_l4zz ...
    I_l5xx I_l5xy I_l5xz I_l5yy I_l5yz I_l5zz ...
    I_l6xx I_l6xy I_l6xz I_l6yy I_l6yz I_l6zz ...
    r_c1x r_c1y r_c1z r_c2x r_c2y r_c2z r_c3x r_c3y r_c3z ...
    r_c4x r_c4y r_c4z r_c5x r_c5y r_c5z r_c6x r_c6y r_c6z...
    k_r1 k_r2 k_r3 k_r4 k_r5 k_r6 ...
    I_m1_m1 I_m2_m2 I_m3_m3 I_m4_m4 I_m5_m5 I_m6_m6 real;

I_1_l1 = [I_l1xx -I_l1xy -I_l1xz;
          -I_l1xy I_l1yy -I_l1yz;
          -I_l1xz -I_l1yz I_l1zz];      

I_2_l2 = [I_l2xx -I_l2xy -I_l2xz;
          -I_l2xy I_l2yy -I_l2yz;
          -I_l2xz -I_l2yz I_l2zz];
      
I_3_l3 = [I_l3xx -I_l3xy -I_l3xz;
          -I_l3xy I_l3yy -I_l3yz;
          -I_l3xz -I_l3yz I_l3zz];

I_4_l4 = [I_l4xx -I_l4xy -I_l4xz;
          -I_l4xy I_l4yy -I_l4yz;
          -I_l4xz -I_l4yz I_l4zz];
      
I_5_l5 = [I_l5xx -I_l5xy -I_l5xz;
          -I_l5xy I_l5yy -I_l5yz;
          -I_l5xz -I_l5yz I_l5zz];
      
I_6_l6 = [I_l6xx -I_l6xy -I_l6xz;
          -I_l6xy I_l6yy -I_l6yz;
          -I_l6xz -I_l6yz I_l6zz];
   
%%%%%%%%%%%%%%%%%% SCHUNK Arm Properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l1 = .205;
l2 = .350;
l3 = .305;
l4 = .082;

T_1 = [ cos(q1),  0, -sin(q1),  0;
    sin(q1),  0,  cos(q1),  0;
    0, -1,        0, l1;
    0,  0,        0,  1];
       
T_2 = [ cos(q1)*sin(q2), cos(q1)*cos(q2), -sin(q1), l2*cos(q1)*sin(q2);
    sin(q1)*sin(q2), cos(q2)*sin(q1),  cos(q1), l2*sin(q1)*sin(q2);
    cos(q2),        -sin(q2),        0,    l1 + l2*cos(q2);
    0,               0,        0,                  1];
                       
T_3 = [ cos(q2 + q3)*cos(q1), -sin(q1), sin(q2 + q3)*cos(q1), l2*cos(q1)*sin(q2);
    cos(q2 + q3)*sin(q1),  cos(q1), sin(q2 + q3)*sin(q1), l2*sin(q1)*sin(q2);
    -sin(q2 + q3),        0,         cos(q2 + q3),    l1 + l2*cos(q2);
    0,        0,                    0,                  1];
                
T_4 = [ cos(q1)*cos(q2)*cos(q3)*cos(q4) - sin(q1)*sin(q4) - cos(q1)*cos(q4)*sin(q2)*sin(q3), -sin(q2 + q3)*cos(q1), cos(q1)*sin(q2)*sin(q3)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q4)*sin(q1), cos(q1)*(l3*sin(q2 + q3) + l2*sin(q2));
    cos(q1)*sin(q4) + cos(q4)*(sin(q1 + q2 + q3)/2 + sin(q1 - q2 - q3)/2), -sin(q2 + q3)*sin(q1), cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4), sin(q1)*(l3*sin(q2 + q3) + l2*sin(q2));
    -sin(q2 + q3)*cos(q4),         -cos(q2 + q3),                                                                sin(q2 + q3)*sin(q4),      l1 + l3*cos(q2 + q3) + l2*cos(q2);
    0,                     0,                                                                                   0,                                      1];
   
T_5 = [ - (sin(q5)*(sin(q1 + q2 + q3) - sin(q1 - q2 - q3)))/2 - cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)), cos(q1)*sin(q2)*sin(q3)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q4)*sin(q1), cos(q5)*(sin(q1 + q2 + q3)/2 - sin(q1 - q2 - q3)/2) - sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)), cos(q1)*(l3*sin(q2 + q3) + l2*sin(q2));
    sin(q5)*(cos(q1 + q2 + q3)/2 - cos(q1 - q2 - q3)/2) + cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3)), cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4), sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3)) - (cos(q5)*(cos(q1 + q2 + q3) - cos(q1 - q2 - q3)))/2, sin(q1)*(l3*sin(q2 + q3) + l2*sin(q2));
    - cos(q2 + q3)*sin(q5) - sin(q2 + q3)*cos(q4)*cos(q5),                                                                sin(q2 + q3)*sin(q4),                                                                                                 cos(q2 + q3)*cos(q5) - sin(q2 + q3)*cos(q4)*sin(q5),      l1 + l3*cos(q2 + q3) + l2*cos(q2);
    0,                                                                                   0,                                                                                                                                                   0,                                      1];
             
T_6 = [ - cos(q6)*(cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)) + sin(q5)*(sin(q1 + q2 + q3)/2 - sin(q1 - q2 - q3)/2)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4)), sin(q6)*(cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)) + sin(q5)*(sin(q1 + q2 + q3)/2 - sin(q1 - q2 - q3)/2)) - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4)), cos(q5)*(sin(q1 + q2 + q3)/2 - sin(q1 - q2 - q3)/2) - sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)), (l3*(sin(q1 + q2 + q3) - sin(q1 - q2 - q3)))/2 - l4*(sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(q1)*cos(q4)*sin(q2)*sin(q3)) - (cos(q5)*(sin(q1 + q2 + q3) - sin(q1 - q2 - q3)))/2) + l2*cos(q1)*sin(q2);
    cos(q6)*(sin(q5)*(cos(q1 + q2 + q3)/2 - cos(q1 - q2 - q3)/2) + cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3))) + sin(q6)*(cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), cos(q6)*(cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(sin(q5)*(cos(q1 + q2 + q3)/2 - cos(q1 - q2 - q3)/2) + cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3))), sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3)) - (cos(q5)*(cos(q1 + q2 + q3) - cos(q1 - q2 - q3)))/2, l4*(sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q3)) - (cos(q5)*(cos(q1 + q2 + q3) - cos(q1 - q2 - q3)))/2) - (l3*(cos(q1 + q2 + q3) - cos(q1 - q2 - q3)))/2 + l2*sin(q1)*sin(q2);
    sin(q2 + q3)*sin(q4)*sin(q6) - cos(q2 + q3)*cos(q6)*sin(q5) - sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6),                                                                                                                                                            sin(q2 + q3)*cos(q6)*sin(q4) + cos(q2 + q3)*sin(q5)*sin(q6) + sin(q2 + q3)*cos(q4)*cos(q5)*sin(q6),                                                                                                 cos(q2 + q3)*cos(q5) - sin(q2 + q3)*cos(q4)*sin(q5),                                                                                                                                  l1 + l3*cos(q2 + q3) + l2*cos(q2) + l4*cos(q2 + q3)*cos(q5) - l4*sin(q2 + q3)*cos(q4)*sin(q5);
    0,                                                                                                                                                                                                                                                             0,                                                                                                                                                   0,                                                                                                                                                                                                                              1];
 
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

p0 = [0 0 0]';
p1 = T_1(1:3,4);
p2 = T_2(1:3,4);
p3 = T_3(1:3,4);
p4 = T_4(1:3,4);
p5 = T_5(1:3,4);

p_l1 = p0 + [r_c1x r_c1y r_c1z]';
p_l2 = p1 + [r_c2x r_c2y r_c2z]';
p_l3 = p2 + [r_c3x r_c3y r_c3z]';
p_l4 = p3 + [r_c4x r_c4y r_c4z]';
p_l5 = p4 + [r_c5x r_c5y r_c5z]';
p_l6 = p5 + [r_c6x r_c6y r_c6z]';

J_l1_P1 = cross(z0,(p_l1-p0));
J_l2_P1 = cross(z0,(p_l2-p0));
J_l3_P1 = cross(z0,(p_l3-p0));
J_l4_P1 = cross(z0,(p_l4-p0));
J_l5_P1 = cross(z0,(p_l5-p0));
J_l6_P1 = cross(z0,(p_l6-p0));

J_l2_P2 = cross(z1,(p_l2-p1));
J_l3_P2 = cross(z1,(p_l3-p1));
J_l4_P2 = cross(z1,(p_l4-p1));
J_l5_P2 = cross(z1,(p_l5-p1));
J_l6_P2 = cross(z1,(p_l6-p1));

J_l3_P3 = cross(z2,(p_l3-p2));
J_l4_P3 = cross(z2,(p_l4-p2));
J_l5_P3 = cross(z2,(p_l5-p2));
J_l6_P3 = cross(z2,(p_l6-p2));

J_l4_P4 = cross(z3,(p_l4-p3));
J_l5_P4 = cross(z3,(p_l5-p3));
J_l6_P4 = cross(z3,(p_l6-p3));

J_l5_P5 = cross(z4,(p_l5-p4));
J_l6_P5 = cross(z4,(p_l6-p4));

J_l6_P6 = cross(z5,(p_l6-p5));

J_l1_P = [J_l1_P1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_l2_P = [J_l2_P1    J_l2_P2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_l3_P = [J_l3_P1    J_l3_P2    J_l3_P3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_l4_P = [J_l4_P1    J_l4_P2    J_l4_P3    J_l4_P4 zeros(3,1) zeros(3,1)];
J_l5_P = [J_l5_P1    J_l5_P2    J_l5_P3    J_l5_P4    J_l5_P5 zeros(3,1)];
J_l6_P = [J_l6_P1    J_l6_P2    J_l6_P3    J_l6_P4    J_l6_P5    J_l6_P6];

J_l1_O1 = z0;
J_l2_O1 = z0;
J_l3_O1 = z0;
J_l4_O1 = z0;
J_l5_O1 = z0;
J_l6_O1 = z0;

J_l2_O2 = z1;
J_l3_O2 = z1;
J_l4_O2 = z1;
J_l5_O2 = z1;
J_l6_O2 = z1;

J_l3_O3 = z2;
J_l4_O3 = z2;
J_l5_O3 = z2;
J_l6_O3 = z2;

J_l4_O4 = z3;
J_l5_O4 = z3;
J_l6_O4 = z3;

J_l5_O5 = z4;
J_l6_O5 = z4;

J_l6_O6 = z5;

J_l1_O = [J_l1_O1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_l2_O = [J_l2_O1    J_l2_O2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_l3_O = [J_l3_O1    J_l3_O2    J_l3_O3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_l4_O = [J_l4_O1    J_l4_O2    J_l4_O3    J_l4_O4 zeros(3,1) zeros(3,1)];
J_l5_O = [J_l5_O1    J_l5_O2    J_l5_O3    J_l5_O4    J_l5_O5 zeros(3,1)];
J_l6_O = [J_l6_O1    J_l6_O2    J_l6_O3    J_l6_O4    J_l6_O5    J_l6_O6];

p_m1 = p0;
p_m2 = p1;
p_m3 = p2;
p_m4 = p3;
p_m5 = p4;
p_m6 = p5;

J_m1_P1 = cross(z0,(p_m1-p0));
J_m2_P1 = cross(z0,(p_m2-p0));
J_m3_P1 = cross(z0,(p_m3-p0));
J_m4_P1 = cross(z0,(p_m4-p0));
J_m5_P1 = cross(z0,(p_m5-p0));
J_m6_P1 = cross(z0,(p_m6-p0));

J_m2_P2 = cross(z1,(p_m2-p1));
J_m3_P2 = cross(z1,(p_m3-p1));
J_m4_P2 = cross(z1,(p_m4-p1));
J_m5_P2 = cross(z1,(p_m5-p1));
J_m6_P2 = cross(z1,(p_m6-p1));

J_m3_P3 = cross(z2,(p_m3-p2));
J_m4_P3 = cross(z2,(p_m4-p2));
J_m5_P3 = cross(z2,(p_m5-p2));
J_m6_P3 = cross(z2,(p_m6-p2));

J_m4_P4 = cross(z3,(p_m4-p3));
J_m5_P4 = cross(z3,(p_m5-p3));
J_m6_P4 = cross(z3,(p_m6-p3));

J_m5_P5 = cross(z4,(p_m5-p4));
J_m6_P5 = cross(z4,(p_m6-p4));

J_m6_P6 = cross(z5,(p_m6-p5));

J_m1_P = [J_m1_P1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_m2_P = [J_m2_P1    J_m2_P2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_m3_P = [J_m3_P1    J_m3_P2    J_m3_P3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_m4_P = [J_m4_P1    J_m4_P2    J_m4_P3    J_m4_P4 zeros(3,1) zeros(3,1)];
J_m5_P = [J_m5_P1    J_m5_P2    J_m5_P3    J_m5_P4    J_m5_P5 zeros(3,1)];
J_m6_P = [J_m6_P1    J_m6_P2    J_m6_P3    J_m6_P4    J_m6_P5    J_m6_P6];

z_m1 = z0;
z_m2 = z1;
z_m3 = z2;
z_m4 = z3;
z_m5 = z4;
z_m6 = z5;

J_m1_O1 = k_r1*z_m1;
J_m2_O1 = J_l2_O1;
J_m3_O1 = J_l3_O1;
J_m4_O1 = J_l4_O1;
J_m5_O1 = J_l5_O1;
J_m6_O1 = J_l6_O1;

J_m2_O2 = k_r2*z_m2;
J_m3_O2 = J_l3_O2;
J_m4_O2 = J_l4_O2;
J_m5_O2 = J_l5_O2;
J_m6_O2 = J_l6_O2;

J_m3_O3 = k_r3*z_m3;
J_m4_O3 = J_l4_O3;
J_m5_O3 = J_l5_O3;
J_m6_O3 = J_l6_O3;

J_m4_O4 = k_r4*z_m4;
J_m5_O4 = J_l5_O4;
J_m6_O4 = J_l6_O4;

J_m5_O5 = k_r5*z_m5;
J_m6_O5 = J_l6_O5;

J_m6_O6 = k_r6*z_m6;

J_m1_O = [J_m1_O1 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_m2_O = [J_m2_O1    J_m2_O2 zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
J_m3_O = [J_m3_O1    J_m3_O2    J_m3_O3 zeros(3,1) zeros(3,1) zeros(3,1)];
J_m4_O = [J_m4_O1    J_m4_O2    J_m4_O3    J_m4_O4 zeros(3,1) zeros(3,1)];
J_m5_O = [J_m5_O1    J_m5_O2    J_m5_O3    J_m5_O4    J_m5_O5 zeros(3,1)];
J_m6_O = [J_m6_O1    J_m6_O2    J_m6_O3    J_m6_O4    J_m6_O5    J_m6_O6];

R_m1 = R_1;
R_m2 = R_2;
R_m3 = R_3;
R_m4 = R_4;
R_m5 = R_5;
R_m6 = R_6;

B1 = m_l1*(J_l1_P'*J_l1_P) + J_l1_O'*R_1*I_1_l1*R_1'*J_l1_O + m_m1*(J_m1_P'*J_m1_P) + J_m1_O'*R_m1*I_m1_m1*R_m1'*J_m1_O;
B2 = m_l2*(J_l2_P'*J_l2_P) + J_l2_O'*R_2*I_2_l2*R_2'*J_l2_O + m_m2*(J_m2_P'*J_m2_P) + J_m2_O'*R_m2*I_m2_m2*R_m2'*J_m2_O;
B3 = m_l3*(J_l3_P'*J_l3_P) + J_l3_O'*R_3*I_3_l3*R_3'*J_l3_O + m_m3*(J_m3_P'*J_m3_P) + J_m3_O'*R_m3*I_m3_m3*R_m3'*J_m3_O;
B4 = m_l4*(J_l4_P'*J_l4_P) + J_l4_O'*R_4*I_4_l4*R_4'*J_l4_O + m_m4*(J_m4_P'*J_m4_P) + J_m4_O'*R_m4*I_m4_m4*R_m4'*J_m4_O;
B5 = m_l5*(J_l5_P'*J_l5_P) + J_l5_O'*R_5*I_5_l5*R_5'*J_l5_O + m_m5*(J_m5_P'*J_m5_P) + J_m5_O'*R_m5*I_m5_m5*R_m5'*J_m5_O;
B6 = m_l6*(J_l6_P'*J_l6_P) + J_l6_O'*R_6*I_6_l6*R_6'*J_l6_O + m_m6*(J_m6_P'*J_m6_P) + J_m6_O'*R_m6*I_m6_m6*R_m6'*J_m6_O;

B = B1 + B2 + B3 + B4 + B5 + B6;
    
q = [q1 q2 q3 q4 q5 q6]';
dq = [dq1 dq2 dq3 dq4 dq5 dq6]';
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6]';
T = 1/2 * dq' * B * dq;

g_0 = [0 0 -g]';
U_l1 = -m_l1*g_0'*p_l1;
U_l2 = -m_l2*g_0'*p_l2;
U_l3 = -m_l3*g_0'*p_l3;
U_l4 = -m_l4*g_0'*p_l4;
U_l5 = -m_l5*g_0'*p_l5;
U_l6 = -m_l6*g_0'*p_l6;

U_m1 = -m_m1*g_0'*p_m1;
U_m2 = -m_m2*g_0'*p_m2;
U_m3 = -m_m3*g_0'*p_m3;
U_m4 = -m_m4*g_0'*p_m4;
U_m5 = -m_m5*g_0'*p_m5;
U_m6 = -m_m6*g_0'*p_m6;

U = U_l1 + U_l2 + U_l3 + U_l4 + U_l5 + U_l6 + U_m1 + U_m2 + U_m3 + U_m4 + U_m5 + U_m6;

state = [q' dq']';
dstate = [dq' ddq']';

L = T-U;

L_dq = jacobian(L,dq)';
L_dq_t = jacobian(L_dq,state)*dstate;
L_q = jacobian(L,q)';

Eq_l = L_dq_t - L_q;
M = jacobian(Eq_l,ddq);
n = Eq_l - M*ddq;
