function plotCoord_update(f, p, R, scale)
d = (plotCoord_points(p, R, scale));

x = d(1, :);
y = d(2, :);
z = d(3, :);

f.XData = [x(:), x(:)];
f.YData = [y(:), y(:)];
f.ZData = [z(:), z(:)];
end