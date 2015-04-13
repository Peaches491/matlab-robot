clc; close all; clear all;
addpath 'kinematics/'
addpath 'Plotting/'
addpath 'Robot/'

%% DH for Three link planner arm
syms theta1 theta2; % theta3;
syms l1 l2; % l3;
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
syms l1 lc1 l2 lc2;

pos1 = [-(l1-lc1); 0; 0; 1];
pos2 = [-(l2-lc2); 0; 0; 1];

Jm1 = simplify(r.Jacobian('end_link', 1, 'position', pos1));
Jm2 = simplify(r.Jacobian('end_link', 2, 'position', pos2));

%% Calculate the Kinetic and Potential Energy For each point mass 
syms theta1dot theta2dot;
syms m1 m2 mL;

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

K1 = simplify(1/2 * [theta1dot;theta2dot].' * ...
    m1 * (Jvm1.' * Jvm1)...
    * [theta1dot;theta2dot] );



K2 = simplify(1/2 * [theta1dot;theta2dot].' * ...
    m2 * (Jvm2.' * Jvm2)...
    * [theta1dot;theta2dot] );

K3 = simplify(1/2 * [theta1dot;theta2dot].' * ...
    (mL * (Jv.' * Jv)) *...
    [theta1dot;theta2dot] );

K = K1 + K2 + K3;

L = simplify(K - P);

%% Lagrand Equations Of Motion 
syms theta1t(t) theta2t(t) theta1dotdot theta2dotdot
q = [theta1 theta2];
qdot = [theta1dot theta2dot];
qdotdot = [theta1dotdot theta2dotdot];

qt = [theta1t(t) theta2t(t)];
qtdot = diff(qt);
qtdotdot = diff(qtdot);

%% Repeat the block for each joint variable 
A1 = diff(L, theta1dot);
A1t = subs(A1, [q qdot], [qt qtdot]);
B1 = diff(L, theta1);
B1t = subs(B1, [q qdot], [qt qtdot]);

Tau1t = diff(A1t, t) - B1t;
Tau1 = simplify(subs(Tau1t, [qt qtdot qtdotdot], [q qdot qdotdot]));
%%

A2 = diff(L, theta2dot);
A2t = subs(A2, [q qdot], [qt qtdot]);
B2 = diff(L, theta2);
B2t = subs(B2, [q qdot], [qt qtdot]);

Tau2t = diff(A2t, t) - B2t;
Tau2 = simplify(subs(Tau2t, [qt qtdot qtdotdot], [q qdot qdotdot]));

%% Mass Matrix 

M11 = simplify(Tau1 -subs(Tau1, theta1dotdot, 0)) / theta1dotdot;
M12 = simplify(Tau1 -subs(Tau1, theta2dotdot, 0)) / theta2dotdot;
M21 = simplify(Tau2 -subs(Tau2, theta1dotdot, 0)) / theta1dotdot;
M22 = simplify(Tau2 -subs(Tau2, theta2dotdot, 0)) / theta2dotdot;

% Find all terms that don't depend on derivatives of DoFs. By zeroing out% everything that does
G1 = simplify(subs(Tau1,{theta1dotdot,theta1dot,theta2dotdot,theta2dot},...
    {0, 0, 0, 0}));
G2 = simplify(subs(Tau2,{theta1dotdot,theta1dot,theta2dotdot,theta2dot},...
    {0, 0, 0, 0}));

% Find all terms no accounted for in M and G
V1 = simplify(Tau1 -(M11 * theta1dotdot + M12 * theta2dotdot + G1));
V2 = simplify(Tau2 -(M21 * theta1dotdot + M22 * theta2dotdot + G2));

Tau = [M11, M12; M21, M22] * qdotdot' + [V1; V2] + [G1; G2];

M = [M11, M12; M21, M22];

simplify(M11 == l1^2*m2 + l1^2*mL + l2^2*mL + lc1^2*m1 + lc2^2*m2 + 2*l1*l2*mL*cos(theta2) + 2*l1*lc2*m2*cos(theta2));
