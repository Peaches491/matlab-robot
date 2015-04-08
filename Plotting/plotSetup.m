function plotSetup (size, a, e, proj)
    cla();
    axis equal;
    grid on;
    hold on;
    axis([-size, size, -size, size, -0.350, 1-0.350]);
    view(a, e);
    camproj(proj);
end 