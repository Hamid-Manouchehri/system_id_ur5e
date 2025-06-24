%% Symbolic RNEA inverse dynamics for UR5e robot
% Define symbolic joint variables
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
syms qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 real
q   = [q1;  q2;  q3;  q4;  q5;  q6];
qd  = [qd1; qd2; qd3; qd4; qd5; qd6];
qdd = [qdd1; qdd2; qdd3; qdd4; qdd5; qdd6];

% Define symbolic inertial parameters (72 total)
syms m1 m2 m3 m4 m5 m6 real
syms cx1 cy1 cz1 cx2 cy2 cz2 cx3 cy3 cz3 cx4 cy4 cz4 cx5 cy5 cz5 cx6 cy6 cz6 real
syms I1xx I1yy I1zz I1xy I1xz I1yz I2xx I2yy I2zz I2xy I2xz I2yz I3xx I3yy I3zz I3xy I3xz I3yz real
syms I4xx I4yy I4zz I4xy I4xz I4yz I5xx I5yy I5zz I5xy I5xz I5yz I6xx I6yy I6zz I6xy I6xz I6yz real
syms Jm1 Jm2 Jm3 Jm4 Jm5 Jm6 Jr1 Jr2 Jr3 Jr4 Jr5 Jr6 gr1 gr2 gr3 gr4 gr5 gr6 real

disp("after symbols")
% UR5e DH parameters (modified DH): [alpha, a, d]
DH = [0,     0,   0.1625;
      -pi/2, 0,        0;
       0,    0.425,    0;
       0,    0.3922,   0.1333;
      pi/2,  0,        0.0997;
      -pi/2, 0,        0.0996];

% Preallocate
z0 = [0;0;1];
w   = sym(zeros(3,7));  wd  = sym(zeros(3,7));
a   = sym(zeros(3,7));  ac  = sym(zeros(3,7));
F   = sym(zeros(3,6));  N   = sym(zeros(3,6));
R   = sym(zeros(3,3,6)); p = sym(zeros(3,6)); pc = sym(zeros(3,6));

% Base initial conditions
w(:,1)   = [0;0;0];
wd(:,1)  = [0;0;0];
a(:,1)   = [0;0;9.81];  % gravity along +Z

% Link COM vectors
c = [cx1, cy1, cz1; cx2, cy2, cz2; cx3, cy3, cz3; cx4, cy4, cz4; cx5, cy5, cz5; cx6, cy6, cz6]';

% Inertia tensors
I = sym(zeros(3,3,6));
for i=1:6
    disp("1st")
    eval(sprintf('I(:,:,i) = [I%dxx, -I%dxy, -I%dxz; -I%dxy, I%dyy, -I%dyz; -I%dxz, -I%dyz, I%dzz];', i,i,i,i,i,i,i,i,i));
end

% Forward recursion
for i = 1:6
    disp("2nd for")
    % Rotation about Z and X axes
    alpha = DH(i,1); a_link = DH(i,2); d = DH(i,3);
    theta = q(i);
    Rz = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    Rx = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
    R(:,:,i) = Rz*Rx;
    p(:,i)   = [a_link; -sin(alpha)*d; cos(alpha)*d];
    pc(:,i)  = R(:,:,i)*c(:,i);

    % Angular velocity & acceleration
    w(:,i+1)  = R(:,:,i)'*w(:,i) + qd(i)*z0;
    wd(:,i+1) = R(:,:,i)'*wd(:,i) + qdd(i)*z0 + cross(w(:,i+1), qd(i)*z0);

    % Linear acceleration of link origin and COM
    a(:,i+1)  = R(:,:,i)'*a(:,i) + cross(wd(:,i+1), p(:,i)) + cross(w(:,i+1), cross(w(:,i+1), p(:,i)));
    ac(:,i)   = a(:,i+1) + cross(wd(:,i+1), pc(:,i)) + cross(w(:,i+1), cross(w(:,i+1), pc(:,i)));

    % Forces and moments on link
    F(:,i) = m1*(i==1)*ac(:,i) + m2*(i==2)*ac(:,i) + m3*(i==3)*ac(:,i) + m4*(i==4)*ac(:,i) + m5*(i==5)*ac(:,i) + m6*(i==6)*ac(:,i);
    N(:,i) = I(:,:,i)*wd(:,i+1) + cross(w(:,i+1), I(:,:,i)*w(:,i+1));
end

% Backward recursion
g = sym(zeros(3,7)); n = sym(zeros(3,7));
g(:,7) = [0;0;0]; n(:,7) = [0;0;0];
for i = 6:-1:1
    disp("3nd for")
    f = R(:,:,i)*g(:,i+1) + F(:,i);
    nn= N(:,i) + R(:,:,i)*n(:,i+1) + cross(pc(:,i), F(:,i)) + cross(p(:,i), R(:,:,i)*g(:,i+1));
    % Joint torque (project onto Z) + motor+gear inertia term
    tau(i,1) = z0' * nn + (Jm1*(i==1) + Jm2*(i==2) + Jm3*(i==3) + Jm4*(i==4) + Jm5*(i==5) + Jm6*(i==6) ...
                 + Jr1*(i==1) + Jr2*(i==2) + Jr3*(i==3) + Jr4*(i==4) + Jr5*(i==5) + Jr6*(i==6)) * qdd(i);
    g(:,i) = f; n(:,i) = nn;
end

% Output torques
disp("befor simplify")
% tau = simplify(tau);
disp("after simplify")

save('rnea_model.mat', 'tau', 'Y')   % tau: symbolic torque vector, Y: symbolic regressor

