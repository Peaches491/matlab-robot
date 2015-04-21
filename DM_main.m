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
syms q1 q2 real;
F1 = [0, q1, 0.6,  0];
F2 = [0, q2, 0.4, 0];

dh_params = [F1; F2];

r = Robot();
r.add_link(F1, 'joint_var', q1);
r.add_link(F2, 'joint_var', q2);

r.add_mass(1, 1, [-(0.3); 0; 0]');
r.add_mass(2, 1, [-(0.2); 0; 0]');
r.add_mass(2, 1, [0; 0; 0]');

%%
clc;
x = r.state_variables(false)';

% No Feedback
u = r.get_joint_torques()';

% P Controller
%q_set = [pi/4, 0];
%P = (eye(2)*10);
%u = P*(r.get_joint_vars(0, false) - q_set)';

% PD Controller
%q_set = [pi/2, 0, 0, 0]';
%P = [10 0 0  0;
%      0 0 0  0;
%      0 0 10 0;
%      0 0 0  0;];
%D = [0  0 0  0;
%     0 10 0  0;
%     0  0 0  0;
%     0  0 0 10;];
%D = (eye(4)*0.2);
%u = P*(x - q_set) + D*x

xd_open = r.state_vector(r.get_joint_torques()');
taus = r.get_joint_torques()'

xd_open'
x
r.state_variables(true)
taus


%% Construct SS Matrices, A, B, and C
clc;

A = sym(zeros(numel(x)));
for eq_idx = 1:numel(xd_open)
    for var_idx = 1:numel(x)
        A(eq_idx, var_idx) = diff(xd_open(eq_idx), x(var_idx));
    end
end
A

B = sym(zeros(numel(x), numel(taus)));
for eq_idx = 1:numel(xd_open)
    for var_idx = 1:numel(taus)
        B(eq_idx, var_idx) = diff(xd_open(eq_idx), taus(var_idx));
    end
end
B

C = eye(4)


%% Linearize Matrices
delta = 0.01;
subs_vec = [x', r.get_joint_torques()]
eq_pt = [0 0 0 0 0 0]
A = eval(subs(A, subs_vec, eq_pt))
B = eval(subs(B, subs_vec, eq_pt))

ss_eq = A*x + B*u;


%% Compare Linearized and Non-Linearized
tmp_state = [delta 0 0 0 0 0]
eval(subs(xd_open, subs_vec, tmp_state))
eval(subs(ss_eq, subs_vec, tmp_state))


%% Simulate

x_0 = [-pi/2 + delta, 0, pi/4, 0]
t_step = 0.01

t = 0:t_step:5.0;
%H = ss(A,B,C,0);
%u_sim = zeros(2, numel(t));
%out = lsim(H, u_sim, t, x_0);

out = zeros(numel(x), numel(t));

current_state = x_0';

f = matlabFunction(ss_eq, 'Vars', [x', taus']);
for i = 1:numel(t)
    current_state = current_state + ...
        f(current_state(1), ...
        current_state(2), ...
        current_state(3), ...
        current_state(4), 0, 0)*t_step;
    out(:, i) = current_state;
end

close all;
%lsim(H, u_sim, t, x_0)

plot(out')
simple_gui2(r, out(1:2:3, :)', t_step);




