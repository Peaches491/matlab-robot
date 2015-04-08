function [ robot ] = Build_Robot( dh_params, sym_vec )
%BUILD_ROBOT Builds a "Robot" object from given DH parameters, and joint
%   variables

robot = [];
          
[rows, columns] = size(dh_params);
for row = 1 : rows
    dh_param = dh_params(row, :);
    mass = 1.0/row;
    I = eye(3);
    T = DHToMatrix_vec(dh_params(row, :));
    joint = sym_vec(row);

    s = struct('DH', dh_param, 'mass', mass, 'inert', I, 'T', T, 'q', joint);
    robot = [robot, s];
end

end

