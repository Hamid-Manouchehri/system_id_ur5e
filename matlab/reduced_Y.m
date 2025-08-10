
% find symbolic form complete regressor
clc
clear all

tmp   = load('../data/mat/Y_sym.mat','Y_sym');
Y_sym = tmp.Y_sym;
Y_fun = @Y_fun;

% todo define trajectory and length
nparam = 66;
%traj_len = nparam+1;
traj_len = 70;
traj = 100*randn(traj_len,18)*pi/180; %6* q,dq,ddq
tol = 1e-4;
Pm = zeros(6*traj_len,nparam);


% subs regressor
tic;
for k = 1:traj_len
    disp(['point: ' num2str(k)]);
    
    q1 = traj(k,1);
    q2 = traj(k,2);
    q3 = traj(k,3);
    q4 = traj(k,4);
    q5 = traj(k,5);
    q6 = traj(k,6);
    dq1 = traj(k,7);
    dq2 = traj(k,8);
    dq3 = traj(k,9);
    dq4 = traj(k,10);
    dq5 = traj(k,11);
    dq6 = traj(k,12);
    ddq1 = traj(k,13);
    ddq2 = traj(k,14);
    ddq3 = traj(k,15);
    ddq4 = traj(k,16);
    ddq5 = traj(k,17);
    ddq6 = traj(k,18);
    Pm( 6*(k-1)+1: 6*k , : ) = subs(Y_sym);  % stacked regressor
    % Pm( 6*(k-1)+1: 6*k , : ) = Y_fun([q1 q2 q3 q4 q5 q6]', [dq1 dq2 dq3 dq4 ...
    % dq5 dq6]', [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6]');
    disp(['time to subs: ',num2str(toc)]);
end

disp(['time to eval Pm: ',num2str(toc)]);

% Pm SVD

[U, S, V] = svd(Pm,0);
rho = rank(S)

V1 = V(:,1:rho);
V2 = V(:,rho+1:nparam);

% init vectors that will contain the indexes of the parameters which
% un-identifiable (Xni) and identifiable (Xi)
Xni = [];
Xi  = [];

% init the regressor related the parameters who are lin. comb. of the
% others
Pm_prime  = [];
nparam_prime = 0;

for k = 1:nparam
    if ( norm(V1(k,:)) < tol )
        Xni = [Xni; k];
    elseif ( norm(V2(k,:)) < tol )
        Xi = [Xi; k];
    else
        Pm_prime = [Pm_prime Pm(:,k)];
        nparam_prime = nparam_prime+1;
    end
end

disp(['Identifiable parameters indexes: ' num2str(Xi.')]);
disp(['Unidentifiable parameters indexes: ' num2str(Xni.')]);

% Pm_prime SVD

[U, S, V] = svd(Pm_prime,0);
rho = rank(S);
nn = nparam_prime;
rr = rho;

V1_prime = V(:,1:rho);
V2_prime = V(:,rho+1:nparam_prime);

V21 = zeros(rho,nparam_prime-rho);
V22 = zeros(nparam_prime-rho,nparam_prime-rho);

oldr = 0;
X1 = zeros(rho,1);
X2 = zeros(nparam_prime-rho,1);

v21_rows = rho;
v22_rows = nparam_prime-rho;

for k = 1:nparam_prime
    
    if (v22_rows > 0)
        V22(v22_rows,:) = V2_prime(nparam_prime+1-k,:);
    end
    
    r = rank(V22)
    
    if ( (r > (oldr+.5)) && ( norm(V2_prime(nparam_prime+1-k,:)) > tol ) )
         X2(v22_rows) = nparam_prime+1-k;
         v22_rows = v22_rows-1;
         oldr = r;
    else
         V21(v21_rows,:) = V2_prime(nn+1-k,:);
         X1(v21_rows) = nparam_prime+1-k;
         v21_rows = v21_rows-1;
    end
    
end


A = -V21/V22;

% Construct the reduced parameter list

%load regressor_completeparameters

% Set a vector of random parameters (complete)
theta = 10*rand(nparam,1);

% Find the torques according to the complete regressor and parameter list
tau_complete = [];
for k=1:traj_len
   tau_complete(6*(k-1)+1:6*k,1) = Pm(6*(k-1)+1:6*k,:)*theta;
end

% Find the indexes related the the parameters gamma_prime_1 and
% gamma_prime_2
cnt = 1;
indX1 = [];
indX2 = [];
for k=1:nparam
    % Check if the index 'k' is related to identifiable, unidentifiable or
    % linear combination parameters 
    if ( (any(Xi==k) == 0) && (any(Xni==k) == 0) )
        % check if the index 'cnt' is related to gamma_prime1 or to 
        % 'gamma_prime_2', then save the index 'k'
        if ( any(X1==cnt) == 1 )
            indX1 = [indX1; k];
        else
            indX2 = [indX2; k];
        end
        cnt=cnt+1;
    end
end

theta_id = theta(Xi);
theta_comb = theta(indX1)+A*theta(indX2);

% Construct the reduced numerical parameter vector
theta_reduced = [theta_id; theta_comb];


% Validate the numerical computation

% Compute the torques from the reduced regressor and parameter vector
for k=1:traj_len
   tau_reduced(6*(k-1)+1:6*k,1) = Pm(6*(k-1)+1:6*k,[Xi; indX1])*theta_reduced;
end

reduce_ok = 0;

tau_diff = norm(tau_complete-tau_reduced)
if tau_diff > 1e-9
    disp('Difference in norm between tau_complete and tau_reduced was bigger than tolerance.')
    disp(['||tau_c-tau_r|| = ' num2str(tau_diff)]);
else
    disp('Difference in norm between tau_complete and tau_reduced was small.')
    disp(['||tau_c-tau_r|| = ' num2str(tau_diff)]);
    reduce_ok = 1;
end
    

% Evaluate the symbolic reduced regressor and parameter vector

if reduce_ok
    
    load p;
    
    theta_id = p(Xi);
    theta_comb = p(indX1)+A*p(indX2);

    % Construct the reduced numerical parameter vector
    theta_reduced = [theta_id; theta_comb];
    regressor_red = Y_sym(:,[Xi; indX1]);
    
    % Save the symbolic results
    save theta_reduced.mat theta_reduced
    save regressor_reduced.mat regressor_red
    
    disp('Reduce regressor and parameter list were saved.');
    disp(['The size of theta_reduced is: ' num2str(length(theta_reduced))]);
    disp(['The size of regressor_red is: ' num2str(size(regressor_red))]);
    
end

