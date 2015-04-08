function [ x ] = plotArm_points(robot, q_vec)
    
    x = zeros(3, robot.num_links() + 1);
    for link = 0 : robot.num_links()
        tmp = robot.TF('end_link', link, 'config', q_vec);
        x(:, link+1) = tmp(1:3, 4)';
    end

end