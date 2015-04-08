clc; clear all; close all;

addpath 'kinematics\'
addpath 'Plotting\'
addpath 'Robot\'

syms q1 q2 q3 q4 q5 q6 l1;
sym_vec = [q1 q2 q3 q4 q5 q6];
dh_params = [  0.000       q1  l1      -pi/2;
               0.000  q2-pi/2  0.250      pi;
               0.000       q3  0.075   -pi/2;
              -0.290       q4  0.000    pi/2;
               0.000       q5  0.000   -pi/2;
              -0.080       q6  0.000      pi; ];


robot = [];
          
[rows, columns] = size(dh_params);
% for row = 1 : rows
%     dh_param = dh_params(row, :);
%     mass = 1.0/row;
%     I = eye(3);
%     T = DHToMatrix_vec(dh_params(row, :));
%     joint = sym_vec(row);
% 
%     s = struct('DH', dh_param, 'mass', mass, 'inert', I, 'T', T, 'q', joint);
%     robot = [robot, s];
% end

%plotSetup(0.90, 148, 15, 'perspective');
%plotarm(robot, [0, 0, 0, 0, 0, 0]);

%TF(robot, 'config', [0, 0, 0, 0, 0, 0])

%simple_gui2(robot);

r = Robot();

[rows, columns] = size(dh_params);
for row = 1 : rows
    dh_param = dh_params(row, :);
    
    mass = 1.0/row;
    joint = sym_vec(row);
    
    r.add_link(dh_param, mass, joint);
end

r.TF('config', [0, 0, 0, 0, 0, 0])

