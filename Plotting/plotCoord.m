function plotCoord(pos, R, scale)

d = plotCoord_points(pos, R, scale);

x = d(1, :);
y = d(2, :);
z = d(3, :);

c = 1:numel(x);      %# colors
surface([x(:), x(:)], [y(:), y(:)], [z(:), z(:)], [c(:), c(:)], ...
    'EdgeColor','flat', 'FaceColor','none', 'LineWidth', 3.0);

c = repmat([1, 0, 0], 7, 1);
c(4:5, :) = repmat([0, 1, 0], 2, 1);
c(6:7, :) = repmat([0, 0, 1], 2, 1);

colormap( c );
end