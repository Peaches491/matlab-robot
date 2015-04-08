function plotArm_update(f_vec, robot, q_vec)

data = plotArm_points(robot, q_vec);

for f_idx = 1 : numel(f_vec)
    f_vec(f_idx).XData = data(1, :);
    f_vec(f_idx).YData = data(2, :);
    f_vec(f_idx).ZData = data(3, :);
end

end