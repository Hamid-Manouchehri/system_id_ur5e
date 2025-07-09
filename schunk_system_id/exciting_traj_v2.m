
function x = exciting_traj()

clc; close all;

regressor_red = [];

load regressor_reduced_33

X = [];

dt = 1;
T = 20;

Wf = 2*pi/10;
N = 4;     % number of frequences
J = 6;     % number of joints
t = 0:dt:T;
landa1 = 1;
landa2 = 1;

x0 = ones(J*2*N,1);
A = [];

for i=1:length(t)
    A = [A ; cal_A(t(i))];
end

% fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)

Q_max = [140*ones(6,1) ; 70*ones(6,1) ; 50*ones(6,1)]*pi/180;
Q_max = repmat(Q_max,2*length(t),1);

opts = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'FiniteDifferenceType','forward', ...
    'UseParallel',true, ...
    'MaxFunctionEvaluations',3e2, ...
    'PlotFcn',{ @optimplotfval, ...           % objective vs iter
                @optimplotfirstorderopt, ...  % KKT optimality
                @optimplotstepsize, ...       % step size
                @optimplotconstrviolation }); % max constraint violation);

[x,fval,exitflag] = fmincon(@regressor_cond,x0,[A;-A],Q_max,[],[],[],[],[],opts);

A = [];
t = 0:0.05:T;

for i = 1:length(t)
    A = [A ; cal_A(t(i))];
end

Q = A*x;

figure;

plot(t,Q(1:18:end),'b'); hold on;
plot(t,Q(2:18:end),'r');
plot(t,Q(3:18:end),'g');
plot(t,Q(4:18:end),'k');
plot(t,Q(5:18:end),'c');
plot(t,Q(6:18:end),'m');

%%% Construct A matrix (can be used for trajectory generation 

%%% and optimization linear condition as well . Ax = Q

    function A = cal_A(t)

        A = zeros(3*J , 2*N*J);
        sub_q = [];
        sub_qd = [];
        sub_qdd = [];

        for l=1:N

            sub_q = [sub_q , 1/(Wf*l)*[sin(Wf*l*t) , -cos(Wf*l*t)] ];
            sub_qd = [sub_qd , [cos(Wf*l*t) , sin(Wf*l*t)] ];
            sub_qdd = [sub_qdd , (Wf*l)*[-sin(Wf*l*t) , cos(Wf*l*t)] ];

        end

        sub_Q = [sub_q ; sub_qd ; sub_qdd];

        for n=1:3
            for i=1:J
                A( (n-1)*J+i , 2*N*(i-1)+1:2*N*i ) = sub_Q(n,:);
            end
        end
    end

    function R = regressor_cond(X)
        Q = A*X;
        R = 0;

        for k=1:length(t)
            [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ...
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
            r34_x, r34_y, r34_z, r45_x, r45_y, r45_z, r56_x, r56_y, r56_z] = Kinematic_Param(Q( (k-1)*18+1 : (k-1)*18+18 ));

            Y_num = subs(regressor_red);
            Y_num = eval(Y_num);
            s = svd(Y_num);
            R = R + ( landa1*cond(Y_num) + landa2/( s(nnz(s))) );

        end
        disp(['R = ' num2str(R)])
    end
end