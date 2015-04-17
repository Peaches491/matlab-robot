function [ q_vec ] = get_joint_vars( robot, order, is_time )

q_vec = [];
for q_idx = 1:robot.num_links()
    q_vec = [q_vec, robot.get_joint_var(q_idx, order, is_time)];
end

end

