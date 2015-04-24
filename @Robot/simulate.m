function [ out ] = simulate(r, x_0, u, eq_pt)
%SIMULATE Summary of this function goes here
%   Detailed explanation goes here

x = r.state_variables(false)';
xd_open = r.state_vector(r.get_joint_torques()');
taus = r.get_joint_torques()';

% Construct SS Matrices, A, B, and C
A = sym(zeros(numel(x)));
for eq_idx = 1:numel(xd_open)
    for var_idx = 1:numel(x)
        A(eq_idx, var_idx) = diff(xd_open(eq_idx), x(var_idx));
    end
end
A;

B = sym(zeros(numel(x), numel(taus)));
for eq_idx = 1:numel(xd_open)
    for var_idx = 1:numel(taus)
        B(eq_idx, var_idx) = diff(xd_open(eq_idx), taus(var_idx));
    end
end
B;

C = eye(numel(x));


% Linearize Matrices
subs_vec = [x', r.get_joint_torques()];

A = eval(subs(A, subs_vec, eq_pt));
B = eval(subs(B, subs_vec, eq_pt));

ss_eq = A*x + B*u;

f = matlabFunction(ss_eq, 'Vars', [x', taus']);

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
out = ode45(@(t, c)(param_expand([c' 0 0], f)), [0, 10.0], x_0, options);
end

