function [ out, C ] = simulate(r, x, eqs, x_0, u, eq_pt, outs)

taus = r.get_joint_torques()';

% Construct SS Matrices, A, B, and C
A = sym(zeros(numel(x)));
for eq_idx = 1:numel(eqs)
    for var_idx = 1:numel(x)
        A(eq_idx, var_idx) = diff(eqs(eq_idx), x(var_idx));
    end
end

B = sym(zeros(numel(x), numel(taus)));
for eq_idx = 1:numel(eqs)
    for var_idx = 1:numel(taus)
        B(eq_idx, var_idx) = diff(eqs(eq_idx), taus(var_idx));
    end
end

C = zeros(nnz(outs), numel(outs));
next_idx = 1;
for c_idx_2 = 1:numel(outs)
    for c_idx_1 = 1:nnz(outs)
        if outs(c_idx_2) == 1 && c_idx_1 == next_idx
            C(c_idx_1, c_idx_2) = 1;
            next_idx = next_idx + 1;
            break;
        end
    end
end

% Linearize Matrices
subs_vec = [x, r.get_joint_torques()];

A = eval(subs(A, subs_vec, eq_pt));
B = eval(subs(B, subs_vec, eq_pt));

A
x
B
u


ax = A*x';
bu = B*u;
ss_eq = ax + bu;

f = matlabFunction(ss_eq, 'Vars', subs_vec);

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);
out = ode45(@(t, c)(param_expand(c', f)), [0, 10.0], x_0, options);
end

