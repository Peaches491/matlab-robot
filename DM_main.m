clc; clear all; close all;

addpath 'kinematics\'
addpath 'Plotting\'
addpath 'Robot\'
import Robot


syms q1 q2 q3 q4 q5 q6;
sym_vec = [q1 q2 q3 q4 q5 q6];
dh_params = [  0.000       q1  0.150      -pi/2;
               0.000  q2-pi/2  0.250      pi;
               0.000       q3  0.075   -pi/2;
              -0.290       q4  0.000    pi/2;
               0.000       q5  0.000   -pi/2;
              -0.080       q6  0.000      pi; ];

r = Robot();

[rows, columns] = size(dh_params);
for row = 1 : rows
    dh_param = dh_params(row, :);
    
    mass = 1.0/row;
    joint = sym_vec(row);
    
    r.add_link(dh_param, mass, 'joint_var', joint);
end



syms theta1 theta2; % theta3;
%sym_vec = [theta1 theta2];
syms l1 l2; % l3;

l1 = 0.25;
l2 = 0.25;
F1 = [0 theta1 0.35 0];
F2 = [0 theta2  l1 0];
F3 = [0      0  l2 0];

r2 = Robot();
r2.add_link(F1, 1, 'joint_var', theta1);
r2.add_link(F2, .5, 'joint_var', theta2);
r2.add_link(F3, .25);

%r.TF('config', [0, 0, 0, 0, 0, 0])
r.Jacobian()

simple_gui2(r);

