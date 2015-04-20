function xdot = state_vector(r, Tau)
qdd = r.get_joint_vars(2, false);
qd = r.get_joint_vars(1, false);

[M, V, G] = r.MassMatrix();

qdd = (-1*M)\(Tau - V.*qd' - G);


xdot = [qd(1); qdd(1); ...
        qd(2); qdd(2);];
end