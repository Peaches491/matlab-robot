function [Tau] = MotionEquations(robot)
q = robot.get_joint_vars(0, false);
qdot = robot.get_joint_vars_dot(1, false);
qdotdot = robot.get_joint_vars_dot(2, false);


end

