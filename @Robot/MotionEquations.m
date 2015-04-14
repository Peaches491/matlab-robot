function [Tau] = MotionEquations(robot)
q = robot.get_joint_vars();
qdot = robot.get_joint_vars_dot();
%qdotdot = robot.get_joint_vars_dot_dot(); % Dan add this to robot 

eval(['syms ', char(q(1)),  't(t)']); % this makes the variables but now I need an array of them

end

