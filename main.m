clc; close all; clear all;
addpath 'kinematics/'
addpath 'Plotting/'
addpath 'Robot/'

%% DH for Three link planner arm
syms theta1 theta2 real; % theta3;
syms theta1d theta2d real;
syms l1 l2 real; % l3;
l1 = 0.8;
l2 = 0.6;
F1 = [0, theta1, l1,  0];
F2 = [0, theta2, l2, 0];


dh_params = [F1; F2];

r = Robot();
r.add_link(F1, 'joint_var', theta1);
r.add_link(F2, 'joint_var', theta2);


 
%% Building Transformations
robot = Build_Robot([F1; F2], [theta1, theta2]);
T01 = simplify(r.TF('end_link', 1));
T02 = simplify(r.TF('end_link', 2));




%% Find Jacobian

J = simplify(r.Jacobian());


%% Find Jacobian for each point mass
%syms l1 lc1 l2 lc2 real;
%syms m1 m2 mL real;

m1 = 1;
m2 = 1;
m3 = 1;
mL = 1;
l1 = 1;
l2 = 1;
lc1 = 0.5;
lc2 = 0.5;

pos1 = [-(l1-lc1); 0; 0; 1];
pos2 = [-(l2-lc2); 0; 0; 1];

r.add_mass(1, m1, [-(l1-lc1); 0; 0]');
r.add_mass(2, m2, [-(l2-lc2); 0; 0]');
r.add_mass(2, mL, [0; 0; 0]');


Jm1 = simplify(r.Jacobian('end_link', 1, 'position', pos1));
Jm2 = simplify(r.Jacobian('end_link', 2, 'position', pos2));

%% Calculate the Kinetic and Potential Energy For each point mass 

syms g

translation0m1 = T01*pos1;
translation0m2 = T02*pos2;
translation0Tip = T02(1:3,4);

Jvm1 = Jm1(1:3,:);
Jvm2 = Jm2(1:3,:);
Jv = J(1:3, :);


P1 = -g *translation0Tip(1) * mL;  %% What direction is gravity in the world frame??? pos X in this example 
P2 = -g*translation0m1(1) * m1;
P3 = -g*translation0m2(1) * m2;

P = simplify(P1 + P2 + P3);

K1 = simplify(1/2 * [theta1d;theta2d].' * ...
    m1 * (Jvm1.' * Jvm1)...
    * [theta1d;theta2d] );



K2 = simplify(1/2 * [theta1d;theta2d].' * ...
    m2 * (Jvm2.' * Jvm2)...
    * [theta1d;theta2d] );

K3 = simplify(1/2 * [theta1d;theta2d].' * ...
    (mL * (Jv.' * Jv)) *...
    [theta1d;theta2d] );

K = K1 + K2 + K3;

L = simplify(K - P);
L2 = r.Lagrangian();

%% Lagrand Equations Of Motion 
syms theta1t(t) theta2t(t) theta1dd theta2dd
q = [theta1 theta2];
qdot = [theta1d theta2d];
qdotdot = [theta1dd theta2dd];

qt = [theta1t(t) theta2t(t)];
qtdot = diff(qt);
qtdotdot = diff(qtdot);

%% Repeat the block for each joint variable 
A1 = diff(L, theta1d);
A1t = subs(A1, [q qdot], [qt qtdot]);
B1 = diff(L, theta1);
B1t = subs(B1, [q qdot], [qt qtdot]);

Tau1t = diff(A1t, t) - B1t;
Tau1 = simplify(subs(Tau1t, [qt qtdot qtdotdot], [q qdot qdotdot]));

A2 = diff(L, theta2d);
A2t = subs(A2, [q qdot], [qt qtdot]);
B2 = diff(L, theta2);
B2t = subs(B2, [q qdot], [qt qtdot]);

Tau2t = diff(A2t, t) - B2t;
Tau2 = simplify(subs(Tau2t, [qt qtdot qtdotdot], [q qdot qdotdot]));

%%
Tau = r.MotionEquations();

%% Mass Matrix 

M11 = simplify(Tau1 -subs(Tau1, theta1dd, 0)) / theta1dd;
M12 = simplify(Tau1 -subs(Tau1, theta2dd, 0)) / theta2dd;
M21 = simplify(Tau2 -subs(Tau2, theta1dd, 0)) / theta1dd;
M22 = simplify(Tau2 -subs(Tau2, theta2dd, 0)) / theta2dd;

% Find all terms that don't depend on derivatives of DoFs. By zeroing out% everything that does
G1 = simplify(subs(Tau1,{theta1dd,theta1d,theta2dd,theta2d},...
    {0, 0, 0, 0}));
G2 = simplify(subs(Tau2,{theta1dd,theta1d,theta2dd,theta2d},...
    {0, 0, 0, 0}));

% Find all terms no accounted for in M and G
V1 = simplify(Tau1 -(M11 * theta1dd + M12 * theta2dd + G1));
V2 = simplify(Tau2 -(M21 * theta1dd + M22 * theta2dd + G2));

Tau = [M11, M12; M21, M22] * qdotdot' + [V1; V2] + [G1; G2];


%simplify(M11 == l1^2*m2 + l1^2*mL + l2^2*mL + lc1^2*m1 + lc2^2*m2 + 2*l1*l2*mL*cos(theta2) + 2*l1*lc2*m2*cos(theta2));



%% State Space format
clc;

[M, V, G] = r.MassMatrix();

M*qdotdot' 
V*qdot

%qdd = -1*inv(M)*(zeros(2, 2) - V*qdot)
%simple_gui2(r);
