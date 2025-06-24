% Example: Minimize f(x) = x1^2 + x2^2  subject to x1 + x2 = 1, x1 >= 0
f = @(x) x(1)^2 + x(2)^2;
x0 = [0, 0];
A = []; b = [];
Aeq = [1 1]; beq = 1;     % Equality: x1 + x2 = 1
lb = [0 0]; ub = [];      % Lower bounds
x_opt = fmincon(f, x0, A, b, Aeq, beq, lb, ub);
disp(x_opt);
