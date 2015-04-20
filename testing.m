clc; clear all;
syms q1 q2 real

x = [(3*cos(q1))/5 + (2*cos(q1)*cos(q2))/5 - (2*sin(q1)*sin(q2))/5, ...
     (3*sin(q1))/5 + (2*cos(q1)*sin(q2))/5 + (2*cos(q2)*sin(q1))/5, ...
                                                                 0]'

f = matlabFunction(x(2), 'Vars', [q1, q2])
                                                             
[X,Y] = meshgrid(-2*pi:.1:2*pi);
Z = zeros(size(X));

for x_val = 1 : size(X, 1)
    for y_val = 1 : size(X, 2)
        Z(x_val, y_val) = f(X(x_val, y_val), Y(x_val, y_val));
    end
end

axis square
surf(X, Y, Z)