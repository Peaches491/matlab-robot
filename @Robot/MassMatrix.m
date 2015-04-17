function [M V G] = MassMatrix(robot)
Tau = robot.MotionEquations();
qdot = robot.get_joint_vars(1, false);
qdotdot = robot.get_joint_vars(2, false);

M = [];


for t = 1 : length(Tau)
    row = [];
    for joint = 1 : length(qdotdot)
        temp = simplify(Tau(t) -subs(Tau(t), qdotdot(joint), 0)) / qdotdot(joint);
        row = [row temp];
    end
    M = [M; row];
end
G = simplify(subs(Tau,[qdot qdotdot],zeros(1,2*length(qdot))));
V = simplify(Tau - (M*qdotdot' + G))

end

