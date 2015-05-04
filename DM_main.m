clc; close all;

addpath 'kinematics\'
addpath 'Plotting\'
addpath 'Robot\'
addpath 'eom2ss\'
import Robot


syms q1 q2 q3 q4 q5 q6  real;
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
    r.add_link(dh_params(row, :), ...
        'joint_var', sym_vec(row));
    
    mass = 1/row + 2.5;
    r.add_mass(row, mass, [0, 0, 0]);
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
r2.add_link(F1, 'joint_var', theta1);
r2.add_link(F2, 'joint_var', theta2);
r2.add_link(F3);

%r.TF('config', [0, 0, 0, 0, 0, 0])
%r.Jacobian()

%r.num_masses(1)
r.Lagrangian();

%simple_gui2(r);



%% Test
clc; clear all;
syms q0 q1 q2 q3 real;
F0 = [0, q0, 0.0,  0.5];
F1 = [0, q1, 0.6,  0];
F2 = [0, q2, 0.4,  0];
F3 = [0, q3, 0.2,  0];

r = Robot();
%r.add_link(F0, 'joint_var', q0);
r.add_link(F1, 'joint_var', q1);
r.add_link(F2, 'joint_var', q2);
%r.add_link(F3, 'joint_var', q3);

r.add_mass(1, 1, [-(0.3); 0; 0]');
r.add_mass(2, 1, [-(0.2); 0; 0]');
%r.add_mass(3, 1, [-(0.1); 0; 0]');
%r.add_mass(4, 1, [-(0.1); 0; 0]');


% Construct Controller
x = r.state_variables(false, false)';
x_0_joints = [-pi/2, -5*pi/6];

q_set = [(ones(1, r.num_links())*pi/4)'];


ctrl_idx = 1;
ctrl_struct = struct('x', [], ...
                     'eqs', [], ...
                     'u', []);

% No Feedback
if false
    u = r.get_joint_torques()';
    u_vec = [u_vec, u];
    u_names{numel(u_names)+1} = 'None';
end

% P Controller
if false
    P = (eye(r.num_links())*10);
    u = P*(r.get_joint_vars(0, false)' - q_set);
    u_vec = [u_vec, u];
    u_names{numel(u_names)+1} = 'P';
end

% PD Controller
if false
    P = eye(r.num_links())*75;
    D = eye(r.num_links())*18.75;
    u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))';
    
    x_0 = zeros(numel(x), 1);
    x_0(1) = x_0_joints(1);
    x_0(3) = x_0_joints(2);
    
    ctrl_struct(ctrl_idx).eqs = r.state_vector(r.get_joint_torques()');
    ctrl_struct(ctrl_idx).x = r.state_variables(false, false);
    ctrl_struct(ctrl_idx).outs = zeros(numel(ctrl_struct(ctrl_idx).x), 1);
    ctrl_struct(ctrl_idx).outs(1) = 1;
    ctrl_struct(ctrl_idx).outs(3) = 1;
    ctrl_struct(ctrl_idx).u = u;
    ctrl_struct(ctrl_idx).x_0 = x_0;
    ctrl_struct(ctrl_idx).name = 'PD';
    
    ctrl_idx = ctrl_idx + 1;
end

% PD+G Controller
if false
    [M, V, G] = r.MassMatrix();
    P = eye(r.num_links())*75;
    D = eye(r.num_links())*18.75;
    u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))' + G;
    u_vec = [u_vec, u];
    u_names{numel(u_names)+1} = 'PD+G';
end

% PD-G Controller
if false
    [M, V, G] = r.MassMatrix();
    P = eye(r.num_links())*75;
    D = eye(r.num_links())*18.75;
    u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))' - G;
    u_vec = [u_vec, u];
    u_names{numel(u_names)+1} = 'PD-G';
end

% G Only Controller  %%%%%%%%% THIS FREAKS OUT. I DON'T KNOW WHY.
if false
    [M, V, G] = r.MassMatrix();
    u = 0.1*G;
    u_vec = [u_vec, u];
    u_names{numel(u_names)+1} = 'G Only';
end

% PID Controller
if true
    pid_x = r.state_variables(false, true);
    ctrl_struct(ctrl_idx).x = pid_x;
    
    pid_eqs = [-1*pid_x(2), pid_x(3), 0, ...
               -1*pid_x(5), pid_x(6), 0, ];
    v = r.state_vector(r.get_joint_torques()');
    pid_eqs(3) = v(2);
    pid_eqs(6) = v(4);
    ctrl_struct(ctrl_idx).eqs = pid_eqs;
    
    pid_q_set = zeros(size(pid_x, 2), 1);
    pid_q_set(2) = q_set(1);
    pid_q_set(5) = q_set(2);
     
    k_p = 45;
    k_i = -20.8;
    k_d = 3.0;
    
    %[M, V, G] = r.MassMatrix();
    K = zeros(2, numel(pid_x));
    K(1, 1) = k_i;
    K(1, 2) = k_p;
    K(1, 3) = k_d;
    K(2, 4) = k_i;
    K(2, 5) = k_p;
    K(2, 6) = k_d;
    
    u = K*(pid_x' - pid_q_set);
    %u_vec = [u_vec, u];
    %u_names{numel(u_names)+1} = 'PID';
    
    ctrl_struct(ctrl_idx).u = u;
    
    x_0 = zeros(numel(pid_x), 1);
    x_0(2) = x_0_joints(1);
    x_0(5) = x_0_joints(2);
    ctrl_struct(ctrl_idx).x_0 = x_0;
    ctrl_struct(ctrl_idx).outs = [0, 1, 0, 0, 1, 0];
    ctrl_struct(ctrl_idx).desired = q_set;
    ctrl_struct(ctrl_idx).outs = [1, 1, 1, 1, 1, 1];
    ctrl_struct(ctrl_idx).desired = pid_q_set;
    ctrl_struct(ctrl_idx).name = {'I1', 'P1', 'D1', 'I2', 'P2', 'D2'};
    
    ctrl_idx = ctrl_idx+1;
end

ctrl_struct(1)

size(ctrl_struct)

close all;
%profile on;
for u_idx = 1:numel(ctrl_struct)
    u = ctrl_struct(u_idx).u
    x = ctrl_struct(u_idx).x
    outs = ctrl_struct(u_idx).outs
    x_0 = ctrl_struct(u_idx).x_0
    eqs = ctrl_struct(u_idx).eqs
    desired = ctrl_struct(u_idx).desired'
    
    subs_vec = [x, r.get_joint_torques()]
    eq_pt = zeros(size(subs_vec));
    eq_pt(2) = pi;

    
    % Simulate
    [out, C] = r.simulate(x, eqs, x_0, u, eq_pt, outs);

    % Calculate Errors   
    out_y = (C*out.y)';
    err = bsxfun(@minus, out_y, desired); 
    %err = sqrt(sum(err.^2, 2));
    final_error = out_y(end, :) - desired
    
    % Plot Errors
    hold on;
    plot(out.x', err)
    
    %simple_gui2(r, out.x, out_y, 0.01);
    
    legend(ctrl_struct(u_idx).name)
end
%legend({ctrl_struct.name})

%%
simple_gui2(r, out.x, out.y(1:2:numel(x), :)', 0.01);



