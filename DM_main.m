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
x = r.state_variables(false)';
q_set = (ones(1, r.num_links())*pi/4)';
u_vec = [];
u_names = {};

% No Feedback
%u = r.get_joint_torques()';
%u_vec = [u_vec, u]

% P Controller
%P = (eye(r.num_links())*10);
%u = P*(r.get_joint_vars(0, false)' - q_set);
%u_vec = [u_vec, u];

% PD Controller
P = eye(r.num_links())*75;
D = eye(r.num_links())*18.75;
u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))';
u_vec = [u_vec, u];
u_names{numel(u_names)+1} = 'PD';

% PD+G Controller
[M, V, G] = r.MassMatrix();
P = eye(r.num_links())*75;
D = eye(r.num_links())*18.75;
u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))' + G;
u_vec = [u_vec, u];
u_names{numel(u_names)+1} = 'PD+G';

% PD-G Controller
[M, V, G] = r.MassMatrix();
P = eye(r.num_links())*75;
D = eye(r.num_links())*18.75;
u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))' - G;
u_vec = [u_vec, u];
u_names{numel(u_names)+1} = 'PD-G';

% G Only Controller  %%%%%%%%% THIS FREAKS OUT. I DON'T KNOW WHY.
%[M, V, G] = r.MassMatrix();
%u = 0.1*G;
%u_vec = [u_vec, u];
%u_names{numel(u_names)+1} = 'G Only';

% PID Controller
[M, V, G] = r.MassMatrix();
P = eye(r.num_links())*75;
I = eye(r.num_links())*0.5;
D = eye(r.num_links())*18.75;
u = P*(r.get_joint_vars(0, false)' - q_set) + ...
    I*(r.get_joint_vars(0, false)' - q_set) + ...
    D*(r.get_joint_vars(1, false))';
u_vec = [u_vec, u];
u_names{numel(u_names)+1} = 'PD+G';

x
state_vector_dot = r.state_variables(true)

subs_vec = [x', r.get_joint_torques()];
eq_pt = zeros(size(subs_vec));
eq_pt(1) = pi;

delta = 0.0;
x_0 = zeros(numel(x), 1);
x_0(1) = -pi/2 + delta;
x_0(3) =  -5*pi/6;

size(u_vec)


close all;
profile on;
for u_idx = 1:size(u_vec, 2)  
    u = u_vec(:, u_idx)
    
    % Simulate
    out = r.simulate(x_0, u, eq_pt);

    % Calculate Errors   
    desired = q_set';
    err = bsxfun(@minus, out.y(1:2:end, :)', desired); 
    err = sqrt(sum(err.^2, 2));
    final_error = out.y(1:2:end, end)' - desired
    
    % Plot Errors
    hold on;
    plot(out.x', err)
end
legend(u_names)

%%
simple_gui2(r, out.x, out.y(1:2:numel(x), :)', 0.01);



