function xdot = state_vector(r, taus)
qdd = r.get_joint_vars(2, false);
qd = r.get_joint_vars(1, false);

[M, V, G] = r.MassMatrix()

qdd = -1*(M\(taus - V.*qd' - G));

num_el = numel(qd)+numel(qdd);
xdot = sym(zeros(num_el, 1));
for i = 0:numel(qd)-1
    xdot(2*i+1) = qd(i+1);
    xdot(2*i+2) = qdd(i+1);
end

if sum(isinf(xdot)) > 0
    error('Mass matrix could not be inverted!')
end

end