function [pts] = plotCoord_points(pos, R_orig, scale)
R = R_orig*scale;

pts = repmat(pos, 1, 7);
pts(:, 2) = pos + R(:, 1);
pts(:, 4) = pos + R(:, 2);
pts(:, 6) = pos + R(:, 3);
end