clc; close all; clear all;
addpath 'kinematics/'
addpath 'Plotting/'
addpath 'Robot/'

%% DH for Three link planner arm
syms theta1 theta2; % theta3;
syms l1 l2; % l3;
F1 = [0 theta1 l1 0];
F2 = [0 theta2 l2 0];
% F3 = [0 theta3 l3 0];

dh_params = [F1; F2]



%% Building Transformations
robot = Build_Robot([F1; F2], [theta1, theta2]);
T01 = TFold(robot, 'end_link', 1);
T02 = TFold(robot, 'end_link', 2);
T03 = TFold(robot);


%% Find Jacobian
%Translational Part 
translation03 = T02(1:3,4);
Rotation03 = T02(1:3,1:3);

% Trents Code
Jv2 = [diff(translation03,theta1), diff(translation03,theta2)];

%Rotational Part 
Jw2 = [T01(1:3,3), T02(1:3,3)];  % Or 0 For a prismatic joint 

%6DOF Jacobian
J2 = simple([Jv2; Jw2]);

% My code

J = simple(Jacobian(robot));

%% Find Jacobian for each point mass
syms l1 lc1 l2 lc2;

pos1 = [-(l1-lc1); 0; 0; 1];
pos2 = [-(l2-lc2); 0; 0; 1];

Jm1 = simple(Jacobian(robot, 'end_link', 1, 'position', pos1));
Jm2 = simple(Jacobian(robot, 'end_link', 2, 'position', pos2));

%% Calculate the Kinetic and Potential Energy For each point mass 
syms theta1dot theta2dot;
syms m1 m2 m3;

syms g

translation0m1 = T01*pos1;
translation0m2 = T02*pos2;
translation03 = T03(1:3,4);

Jvm1 = Jm1(1:3,:);
Jvm2 = Jm2(1:3,:);
Jv = J(1:3, :);


P1 = -g *translation03(1) * m3;
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
    (m3 * (Jv.' * Jv)) *...
    [theta1dot;theta2dot] );

K = K1 + K2 + K3;


