function [Tau] = MotionEquations(robot)
q = robot.get_joint_vars(0, false);
qdot = robot.get_joint_vars(1, false);
qdotdot = robot.get_joint_vars(2, false);

qt = robot.get_joint_vars(0, true);
qtdot = diff(qt);
qtdotdot = diff(qtdot);

L = robot.Lagrangian();
Tau = [];

for joint = 1 : length(q)
    A = diff(L, qdot(joint));
    At = subs(A, [q qdot], [qt qtdot]);
    B = diff(L, q(joint));
    Bt = subs(B, [q qdot], [qt qtdot]);

    Taut = diff(At) - Bt;
    Tau = [Tau; simplify(subs(Taut, [qt qtdot qtdotdot], [q qdot qdotdot]))];

end


