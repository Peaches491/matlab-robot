function [ f ] =  plotArm(robot, q_vec )
x = plotArm_points(robot, q_vec);

f_tmp = plot3(x(1, :), x(2, :), x(3, :), 'Color', [0.5 0.5 0.5], ...
              'LineWidth', 5);

n_plots = 2;
f = f_tmp(ones(1,n_plots),:);

f(2) = scatter3( x(1, :), x(2, :), x(3, :) , 100, [1 0.3 1], 'filled');
end


