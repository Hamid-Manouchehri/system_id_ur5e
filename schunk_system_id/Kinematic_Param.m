function [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ...
    R_10xx, R_10xy, R_10xz, R_10yx, R_10yy, R_10yz, R_10zx, R_10zy, R_10zz, ... 
    R_21xx, R_21xy, R_21xz, R_21yx, R_21yy, R_21yz, R_21zx, R_21zy, R_21zz, ...
    R_32xx, R_32xy, R_32xz, R_32yx, R_32yy, R_32yz, R_32zx, R_32zy, R_32zz, ...
    R_43xx, R_43xy, R_43xz, R_43yx, R_43yy, R_43yz, R_43zx, R_43zy, R_43zz, ...
    R_54xx, R_54xy, R_54xz, R_54yx, R_54yy, R_54yz, R_54zx, R_54zy, R_54zz, ...
    R_65xx, R_65xy, R_65xz, R_65yx, R_65yy, R_65yz, R_65zx, R_65zy, R_65zz, ...
    ddp1_x, ddp1_y, ddp1_z, ddp2_x, ddp2_y, ddp2_z, ddp3_x, ddp3_y, ddp3_z, ...
    ddp4_x, ddp4_y, ddp4_z, ddp5_x, ddp5_y, ddp5_z, ddp6_x, ddp6_y, ddp6_z, ...
    w1_x, w1_y, w1_z, w2_x, w2_y, w2_z, w3_x, w3_y, w3_z, ...
    w4_x, w4_y, w4_z, w5_x, w5_y, w5_z, w6_x, w6_y, w6_z, ...
    dw1_x, dw1_y, dw1_z, dw2_x, dw2_y, dw2_z, dw3_x, dw3_y, dw3_z, ...
    dw4_x, dw4_y, dw4_z, dw5_x, dw5_y, dw5_z, dw6_x, dw6_y, dw6_z, ...
    r01_x, r01_y, r01_z, r12_x, r12_y, r12_z, r23_x, r23_y, r23_z, ...
    r34_x, r34_y, r34_z, r45_x, r45_y, r45_z, r56_x, r56_y, r56_z] = Kinematic_Param(traj)
 
 
    %%%Rotation matrices
    q1 = traj(1);
    q2 = traj(2);
    q3 = traj(3);
    q4 = traj(4);
    q5 = traj(5);
    q6 = traj(6);
    
    ddq1 = traj(13);
    ddq2 = traj(14);
    ddq3 = traj(15);
    ddq4 = traj(16);
    ddq5 = traj(17);
    ddq6 = traj(18);

    l1 = 0;
    l2 = .350;
    l3 = .305;
    l4 = 0;

    % According to IDETC2015 paper
    a_value = [0 l2 0 0 0 0];
    alpha_value = [pi/2 pi pi/2 -pi/2 pi/2 0];
    d_value = [0 0 0 l3 0 0];
    theta_value = [q1 q2+pi/2 q3+pi/2 q4 q5 q6];

    T = zeros(4*6,4);
    for i=1:6
        a = a_value(i);
        alpha = alpha_value(i);
        d = d_value(i);
        theta = theta_value(i);

        A = [ cos(theta) , -sin(theta)*cos(alpha) , sin(theta)*sin(alpha) , a*cos(theta);
          sin(theta) , cos(theta)*cos(alpha) , -cos(theta)*sin(alpha) , a*sin(theta);
          0          , sin(alpha)            , cos(alpha)             , d;
          0          , 0                     , 0                      , 1];

        T(4*(i-1)+1:4*i,:) = A;
    end

    R10 = T(1:3,1:3);
    R21 = T(5:7,1:3);
    R32 = T(9:11,1:3);
    R43 = T(13:15,1:3);
    R54 = T(17:19,1:3);
    R65 = T(21:23,1:3);

    R = R10;
    R_10xx = R(1,1); R_10xy = R(1,2); R_10xz = R(1,3);
    R_10yx = R(2,1); R_10yy = R(2,2); R_10yz = R(2,3);
    R_10zx = R(3,1); R_10zy = R(3,2); R_10zz = R(3,3);

    R = R21;
    R_21xx = R(1,1); R_21xy = R(1,2); R_21xz = R(1,3);
    R_21yx = R(2,1); R_21yy = R(2,2); R_21yz = R(2,3);
    R_21zx = R(3,1); R_21zy = R(3,2); R_21zz = R(3,3);

    R = R32;
    R_32xx = R(1,1); R_32xy = R(1,2); R_32xz = R(1,3);
    R_32yx = R(2,1); R_32yy = R(2,2); R_32yz = R(2,3);
    R_32zx = R(3,1); R_32zy = R(3,2); R_32zz = R(3,3);

    R = R43;
    R_43xx = R(1,1); R_43xy = R(1,2); R_43xz = R(1,3);
    R_43yx = R(2,1); R_43yy = R(2,2); R_43yz = R(2,3);
    R_43zx = R(3,1); R_43zy = R(3,2); R_43zz = R(3,3);

    R = R54;
    R_54xx = R(1,1); R_54xy = R(1,2); R_54xz = R(1,3);
    R_54yx = R(2,1); R_54yy = R(2,2); R_54yz = R(2,3);
    R_54zx = R(3,1); R_54zy = R(3,2); R_54zz = R(3,3);

    R = R65;
    R_65xx = R(1,1); R_65xy = R(1,2); R_65xz = R(1,3);
    R_65yx = R(2,1); R_65yy = R(2,2); R_65yz = R(2,3);
    R_65zx = R(3,1); R_65zy = R(3,2); R_65zz = R(3,3);
    
    
    %%% Displacement between frame i-1 and i

    r01_x = T(1,4);
    r01_y = T(2,4);
    r01_z = T(3,4);
    
    r12_x = T(5,4);
    r12_y = T(6,4);
    r12_z = T(7,4);
    
    r23_x = T(9,4);
    r23_y = T(10,4);
    r23_z = T(11,4);
    
    r34_x = T(13,4);
    r34_y = T(14,4);
    r34_z = T(15,4);
    
    r45_x = T(17,4);
    r45_y = T(18,4);
    r45_z = T(19,4);
    
    r56_x = T(21,4);
    r56_y = T(22,4);
    r56_z = T(23,4);
    
    %%% Linear accelerations, angular velocities and accelerations
    
    w = zeros(3,6);
    wdot = zeros(3,6);
    pddot = zeros(3,6);
    
    g = 9.81;
    g0 = [0;0;-g];
    w_0 = zeros(3,1);
    wdot_0 = zeros(3,1);
    pddot_0 = zeros(3,1) - g0;
    
    z0 = [0;0;1];
    
    for k = 1:6
        R = T(4*(k-1)+1:4*k-1,1:3);  % frame k with respect to frame k-1
        r = R'* T(4*(k-1)+1:4*k-1,4);    % r^k_(k-1,k)
        if (k==1)
            w(:,k) = R' * (w_0 + traj(6+k)*z0);
            wdot(:,k) = R' * ( wdot_0 + traj(12+k)*z0 + traj(6+k)*cross(wdot_0,z0) );
            pddot(:,k) = R'*pddot_0 + cross(wdot(:,k),r) + cross(w(:,k), cross(w(:,k),r) );
        else
            w(:,k) = R' * (w(:,k-1) + traj(6+k)*z0);
            wdot(:,k) = R' * ( wdot(:,k-1) + traj(12+k)*z0 + traj(6+k)*cross(wdot(:,k-1),z0) );
            pddot(:,k) = R'*pddot(:,k-1) + cross(wdot(:,k),r) + cross(w(:,k), cross(w(:,k),r) );
        end
    end
    
    % frame origin linear acceleration
    ddp1_x = pddot(1,1); ddp1_y = pddot(2,1); ddp1_z = pddot(3,1);
    ddp2_x = pddot(1,2); ddp2_y = pddot(2,2); ddp2_z = pddot(3,2);
    ddp3_x = pddot(1,3); ddp3_y = pddot(2,3); ddp3_z = pddot(3,3);
    ddp4_x = pddot(1,4); ddp4_y = pddot(2,4); ddp4_z = pddot(3,4);
    ddp5_x = pddot(1,5); ddp5_y = pddot(2,5); ddp5_z = pddot(3,5);
    ddp6_x = pddot(1,6); ddp6_y = pddot(2,6); ddp6_z = pddot(3,6);
    
    % frame origin angular velocity
    w1_x = w(1,1); w1_y = w(2,1); w1_z = w(3,1); 
    w2_x = w(1,2); w2_y = w(2,2); w2_z = w(3,2);
    w3_x = w(1,3); w3_y = w(2,3); w3_z = w(3,3);
    w4_x = w(1,4); w4_y = w(2,4); w4_z = w(3,4);
    w5_x = w(1,5); w5_y = w(2,5); w5_z = w(3,5);
    w6_x = w(1,6); w6_y = w(2,6); w6_z = w(3,6);
    
    % frame origin angular acceleration
    dw1_x = wdot(1,1); dw1_y = wdot(2,1); dw1_z = wdot(3,1); 
    dw2_x = wdot(1,2); dw2_y = wdot(2,2); dw2_z = wdot(3,2);
    dw3_x = wdot(1,3); dw3_y = wdot(2,3); dw3_z = wdot(3,3);
    dw4_x = wdot(1,4); dw4_y = wdot(2,4); dw4_z = wdot(3,4);
    dw5_x = wdot(1,5); dw5_y = wdot(2,5); dw5_z = wdot(3,5);
    dw6_x = wdot(1,6); dw6_y = wdot(2,6); dw6_z = wdot(3,6);
    

    
end