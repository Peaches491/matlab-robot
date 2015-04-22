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
F2 = [0, q2, 0.4, 0];
F3 = [0, q3, 0.2, 0];

r = Robot();
%r.add_link(F0, 'joint_var', q0);
r.add_link(F1, 'joint_var', q1);
r.add_link(F2, 'joint_var', q2);
%r.add_link(F3, 'joint_var', q3);

r.add_mass(1, 1, [-(0.3); 0; 0]');
r.add_mass(2, 1, [-(0.2); 0; 0]');
%r.add_mass(3, 1, [-(0.1); 0; 0]');
%r.add_mass(4, 1, [-(0.1); 0; 0]');

%%
clc;
x = r.state_variables(false)';

% No Feedback
u = r.get_joint_torques()';

% P Controller
q_set = zeros(1, r.num_links());
q_set(1) = pi/4;
P = (eye(r.num_links())*10);
u = P*(r.get_joint_vars(0, false) - q_set)';

% PD Controller
q_set = [pi/2, 0]';
P = eye(r.num_links())*10
D = eye(r.num_links())*2
u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))'

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

C = eye(numel(x))


%% Linearize Matrices
subs_vec = [x', r.get_joint_torques()];
eq_pt = zeros(size(subs_vec));
eq_pt(1) = pi;

A = eval(subs(A, subs_vec, eq_pt));
B = eval(subs(B, subs_vec, eq_pt));

ss_eq = A*x + B*u;


%% Compare Linearized and Non-Linearized
delta = 0.01;
tmp_state = zeros(size(subs_vec));
tmp_state(1) = delta;
tmp_state(3) = delta;
eval(subs(xd_open, subs_vec, tmp_state))
eval(subs(ss_eq, subs_vec, tmp_state))


%%
clc; close all;
x_0 = zeros(numel(x), 1);
x_0(1) = -pi/2 + delta;
x_0(3) =  -5*pi/6;
x_0

ss_eq = A*x + B*u;

q_set = [pi/2, 0]';
P = eye(r.num_links())*80
D = eye(r.num_links())*35
u = P*(r.get_joint_vars(0, false)' - q_set) + D*(r.get_joint_vars(1, false))'

f = matlabFunction(ss_eq, 'Vars', [x', taus']);

%%% THIS IS THE ONLY THING I CANT MAKE WORK FOR ARBITRARY STATE VECTORS =[
ode_f = @(t, current_state) f(current_state(1), current_state(2), ...
    current_state(3),  current_state(4), 0, 0);
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
out = ode45(ode_f, [0, 10.0], x_0, options);
plot(out.y')

simple_gui2(r, out.x, out.y(1:2:numel(x), :)', 0.01);


%% Simulate using LSim
close all;

t_step = 0.01
t = 0:t_step:5.0;

H = ss(A,B,C,0);
u_sim = zeros(2, numel(t));
l_sim_out = lsim(H, u_sim, t, x_0);
lsim(H, u_sim, t, x_0)

%simple_gui2(r, l_sim_out(:, 1:2:numel(x)), t_step);


%% Simulate using Iterated SS Equations
close all;
out = zeros(numel(x), numel(t));

x_0 = [-pi/2 + delta, 0, pi/4, 0]
t_step = 0.0001
t = 0:t_step:10.0;

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

%lsim(H, u_sim, t, x_0)

plot(out')
%simple_gui2(r, out(1:2:3, :)', t_step);



