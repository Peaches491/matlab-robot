function [ x ] = plotArm_points(robot, q_vec)
    
    x = zeros(3, size(robot, 2)+1);
    for link = 0 : size(robot, 2)
        tmp = TF(robot, 'end_link', link, 'config', q_vec);
        x(:, link+1) = tmp(1:3, 4)';
    end

end