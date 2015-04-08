function plotCoordTrans (T, scale)

    R = T(1:3, 1:3);
    p = T(1:3, 4);

    plotCoord(p, R, scale);
end