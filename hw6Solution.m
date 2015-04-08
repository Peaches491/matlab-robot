%% Homework 6
% RBE 3001 Unified Robotics III C-Term 2014

%% Problem 1
% Given a two-link arm in a 2 dimensional space, shown in Figure 1. Two
% degrese of freedom $$\theta_1$ and $$\theta_2$ measured from the positive
% y axis in counter clockwise direction and from along the first link in a
% counter clockwise direction. Arm has lengths of $$\l_1$ and $$\l_2$ and
% point masses $$m_1$ and $$m_2$ at distance $$\l_{c1}$ and $$\l_{c2}$ from
% the start of each link and mass at the end effector $$m_L$
%
% <<Figure1.PNG>>
%


% Instantiate symbolic variables
syms l1 l2 lc1 lc2 m1 m2 mL % constants
syms this theta1 theta2          % variables
%% 
% *a.* Determine forward position kinematics for the arm

%%
% For the first link

% Position
x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);
z1 = 0;
translation01 = [x1; y1; z1];
% Rotation
R111 = cos(theta1); R112 = -sin(theta1); R113 = 0;
R121 = sin(theta1); R122 = cos(theta1);  R123 = 0;
R131 = 0;           R132 = 0;            R133 = 1;
Rotation01 = [R111, R112, R113;
              R121, R122, R123;
              R131, R132, R133];
% Transformation
T01 = [Rotation01, translation01;
    [0,0,0,1]]
%%
% For the second link

% Position
x2 = l2 * cos(theta2);
y2 = l2 * sin(theta2);
z2 = 0;
translation1L = [x2; y2; z2];
% Rotation
R211 = cos(theta2); R212 = -sin(theta2); R213 = 0;
R221 = sin(theta2); R222 = cos(theta2);  R223 = 0;
R231 = 0;           R232 = 0;            R233 = 1;
Rotation1L = [R211, R212, R213;
              R221, R222, R223;
              R231, R232, R233];
% Transformation
T1L = [Rotation1L, translation1L;
    [0,0,0,1]]
%%
% Combine
T0L = simplify(T01 * T1L);
translation0L = T0L(1:3,4)
Rotation0L = T0L(1:3,1:3)
%% 
% *b.* Determine forward velocity kinematics with respect to arm tip using
% Jacobian.
% 
% $$\xi = \left[ \matrix{ v^n_0 \cr \omega^n_0 } \right] = J\dot{q}$
%
% $$J = \left[ \begin{array}{c} J_V  \\ J_\omega \end{array} \right]$
%
%%
% Find Jacobian using partial derivatives
%
% $$J_V = \left[\frac{\partial \vec{x}_t(\vec{q})}{\partial q_1}
% \frac{\partial \vec{x}_t(\vec{q})}{\partial q_2} .... \frac{\partial
% \vec{x}_t(\vec{q})}{\partial q_i}\right]$
%
Jv = [ diff(translation0L,theta1),    diff(translation0L,theta2) ]
%%
% _Or_ Find Jacobian using |jacobian| function
Jv = jacobian(translation0L,[theta1,theta2])
%%
% _Or_ Find Jacobian using alternate method From Spong
%
% $$J_V = [z_{i-1} \times (o_{n} - o_{i-1})]$
Jv = [[-translation0L(2); translation0L(1); 0],... Contribution from theta1
      [-translation0L(2) + translation01(2);...     Contribution from theta2
       translation0L(1) - translation01(1); 0] ]   
%%
% Angular velocity
%
% $$J_\omega = [z_{i-1}]$
Jw = [Rotation01(1:3,3),...  Contribution from theta1, axis 0
    Rotation0L(1:3,3)]     % Contribution from theta2, axis 1
%%
% Combine
syms theta1dot theta2dot

J = [Jv; Jw]
Xi = J * [theta1dot;theta2dot]
%% 
% For mass 1
%
% $$m_1$ location can be found a few different ways. In this solution,
% $$m_1$'s location is given in reference frame 1 and then transformed to
% frame 0 using the transformation from part a.
syms lc1
translation0m1 = simplify(T01 * [0; 0; 0; 1])% m1 location
% Linear velocity using cross product
Jvm1 = [diff(translation0m1, theta1), ...  Contribution from theta1
    diff(translation0m1, theta2)];       % Contribution from theta2
       
% Angular velocity
Jwm1 = [Rotation01(1:3,3),...  Contribution from theta1
    [0;0;0]];                % Contribution from theta2
% Combine
Jm1 = [Jvm1; Jwm1]
%%
% For mass 2
%
% $$m_2$ location can be found a few different ways. In this solution,
% $$m_2$'s location is given in reference frame L and then transformed to
% frame 0 using the transformation from part a.
syms lc2
translation0m2 = simplify(T0L * [0; 0; 0; 1])% m2 location
% Linear velocity using cross product
Jvm2 = [diff(translation0m2, theta1), ...  Contribution from theta1
    diff(translation0m2, theta2)];       % Contribution from theta2

% Angular velocity
Jwm2 = [Rotation01(1:3,3),...  Contribution from theta1
    Rotation0L(1:3,3)];      % Contribution from theta2
% Combine
Jm2 = [Jvm2; Jwm2]
%%
% *c.* Solve for the arm dynamics using the Euler-Lagrange approach

%% 
% Derive the kinetic energy of the arm
%
% $$K = \frac{1}{2}\dot{q}^T \left[ \sum_{i=1}^{n} \left\{
% m_iJ_{v_i}(q)^TJ_{v_i}(q) +
% J_{\omega_i}(q)^TR_i(q)I_iR_i^T(q)J_{\omega_i}(q)\right\} \right]\dot{q}$
%
% With point mass, $$\mathit{I}=0$
%
% $$K = \frac{1}{2}\dot{q}^T \left[ \sum_{i=1}^{n} \left\{
% m_iJ_{v_i}(q)^TJ_{v_i}(q)\right\} \right]\dot{q}$

K = simplify(1/2 * [theta1dot;theta2dot].' * ...
    (mL * (Jv.' * Jv) +...
    m1 * (Jvm1.' * Jvm1) +...
    m2 * (Jvm2.' * Jvm2)) *...
    [theta1dot;theta2dot] );

K1 = simplify(1/2 * [theta1dot;theta2dot].' * ...
    m1 * (Jvm1.' * Jvm1)...    
    * [theta1dot;theta2dot] )


K2 = simplify(1/2 * [theta1dot;theta2dot].' * ...
    m2 * (Jvm2.' * Jvm2) *...
    [theta1dot;theta2dot] )


KL = simplify(1/2 * [theta1dot;theta2dot].' * ...
    (mL * (Jv.' * Jv)) *...
    [theta1dot;theta2dot] )



%% 
% Derive the potential energy of the arm
%
% $$P = m_ig^Tr_c$
%
% $$r_c = y(q)$
%
% $$P_{Total} = P_{CoM} + P_{EE}$
%
syms g

P = simplify(g * -(translation0L(1) * mL + ...
                  translation0m1(1) * m1 + ...
                  translation0m2(1) * m2))

%% 
% Derive the Lagrangian of the arm $$L = K - P$
L = simplify(K - P);

%% 
% Derive the dynamics using Euler-Lagrange
%
% $$\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial
% L}{\partial q_i} = \tau_i$
%%
% Use temporary variables for each of the terms where we take a derivative
% with respect to theta or its derivatives.
%
% $$\frac{d}{dt}A - B = \tau$
%
% $$A=\frac{\partial L}{\partial \dot{q}}$
%
% $$B=\frac{\partial L}{\partial q}$
syms theta1dotdot theta2dotdot
syms theta1t(t) theta2t(t)

A1 = diff(L,theta1dot);
A1t = subs(A1,{theta1,theta1dot,theta2,theta2dot}...
    ,{theta1t,diff(theta1t(t),t),theta2t,diff(theta2t(t),t)}); % Put in terms of t

A2 = diff(L,theta2dot);
A2t = subs(A2,{theta1,theta1dot,theta2,theta2dot}...
    ,{theta1t,diff(theta1t(t),t),theta2t,diff(theta2t(t),t)}); % Put in terms of t

B1 = diff(L,theta1);
B1t = subs(B1,{theta1,theta1dot,theta2,theta2dot}...
    ,{theta1t,diff(theta1t(t),t),theta2t,diff(theta2t(t),t)}); % In terms of t

B2 = diff(L,theta2);
B2t = subs(B2,{theta1,theta1dot,theta2,theta2dot}...
    ,{theta1t,diff(theta1t(t),t),theta2t,diff(theta2t(t),t)}); % In terms of t

Tau1t = diff(A1t,t) - B1t; % Compute in terms of t

Tau2t = diff(A2t,t) - B2t; % Compute in terms of t

Tau1 = simplify(subs(Tau1t, ...
    {theta1t,diff(theta1t(t),t),diff(theta1t(t),t,t),...
    theta2t,diff(theta2t(t),t),diff(theta2t(t),t,t)},...
    {theta1,theta1dot,theta1dotdot,theta2,theta2dot,theta2dotdot})); % Put in terms independent of time

Tau2 = simplify(subs(Tau2t, ...
    {theta1t,diff(theta1t(t),t),diff(theta1t(t),t,t),...
    theta2t,diff(theta2t(t),t),diff(theta2t(t),t,t)},...
    {theta1,theta1dot,theta1dotdot,theta2,theta2dot,theta2dotdot})); % Put in terms independent of time
%% 
% Because the problem asks for standard form, we must isolate the terms
% that correspond to different physical interpretations of the cause of the
% torque.
%
% $$\vec{\tau} = M(\vec{q})\vec{\ddot{q}} + V(\vec{q},\vec{\dot{q}}) +
% G(\vec{q})$
%
% $$\vec{q}$ is a vector of all of the setable degrees of freedom. In this
% case it consists of the $$\theta_1$ and $$\theta_2$
%
% $$M(\vec{q})$ is the Inertia and consists of all of the coefficients on
% second derivatives of our degrees of freedom
%
% $$V(\vec{q},\vec{\dot{q}})$ is the Coriolis/Centripital Coupling term.
% This consists of all terms that include a first derivative of our degrees
% of freedom.
%
% $$G(\vec{q})$ is the Gravity term. This depends only on the degrees of
% freedom and not on their derivatives.

% Find all terms that are coefficients of second derivatives of DoFs
M11 = simplify(Tau1 - subs(Tau1, theta1dotdot, 0)) / theta1dotdot 
M12 = simplify(Tau1 - subs(Tau1, theta2dotdot, 0)) / theta2dotdot
M21 = simplify(Tau2 - subs(Tau2, theta1dotdot, 0)) / theta1dotdot
M22 = simplify(Tau2 - subs(Tau2, theta2dotdot, 0)) / theta2dotdot

% Find all terms that don't depend on derivatives of DoFs. By zeroing out
% everything that does
G1 = simplify(subs(Tau1,{theta1dotdot,theta1dot,theta2dotdot,theta2dot},...
    {0, 0, 0, 0}))
G2 = simplify(subs(Tau2,{theta1dotdot,theta1dot,theta2dotdot,theta2dot},...
    {0, 0, 0, 0}))

% Find all terms no accounted for in M and G
V1 = simplify(Tau1 - (M11 * theta1dotdot + M12 * theta2dotdot + G1))
V2 = simplify(Tau2 - (M21 * theta1dotdot + M22 * theta2dotdot + G2))

Tau = [M11, M12; M21, M22] * [theta1dotdot; theta2dotdot] + [V1; V2] + [G1; G2];

%% 
% *d.* Write out the inverse dynamics controller
%
%%
% Assuming that the joint torques are the system input, system has form
%
% $$u = M(q)(a_q) + C(q,\dot{q}) + g(q)$
%
% $$a_q = \ddot{\theta}_{desired} - K_pe(t) - K_d\dot{e}(t)$ PD controller
%
% $$u = M(q)(\ddot{\theta}_{desired} - K_pe(t) - K_d\dot{e}(t)) +
% C(q,\dot{q}) + g(q)$
%
% Where the error terms are computed as below
%
% $$e(t) = \theta_{desired} - \theta$
%
% $$\dot{e}(t) = \dot{\theta}_{desired} - \dot{\theta}$
%% 
% *e.* Calculate joint torque for given configurations
%
% where $$\ddot{\theta}_1 = 0, \ddot{\theta}_2 = 0, \dot{\theta}_1 = 0,
% \dot{\theta}_2 = 0$
%% 
% $$\theta_1 = 0^\circ, \theta_2 = 0^\circ$
Taue1 = subs(Tau,...
    {theta1, theta1dot, theta1dotdot, theta2, theta2dot ,theta2dotdot},...
    {0, 0, 0, 0, 0, 0})
%%
% Taking some reasonable values, where the Center of Masses are centered
% and the links have mass 5 kg with a 1 kg payload and links are 0.3 m and
% 0.2 m long. The following torqe estimates are in Nm
eval(subs(Taue1,...
    {l1, l2, lc1, lc2, m1, m2, mL, g},...
    {0.3, 0.2, 0.15, 0.1, 5, 5, 1, 9.82}))
%% 
% $$\theta_1 = 30^\circ, \theta_2 = 0^\circ$
Taue2 = subs(Tau,...
    {theta1, theta1dot, theta1dotdot, theta2, theta2dot ,theta2dotdot},...
    {30/180*pi, 0, 0, 0, 0, 0})
%%
% Taking some reasonable values, where the Center of Masses are centered
% and the links have mass 5 kg with a 1 kg payload and links are 0.3 m and
% 0.2 m long. The following torqe estimates are in Nm
eval(subs(Taue2,...
    {l1, l2, lc1, lc2, m1, m2, mL, g},...
    {0.3, 0.2, 0.15, 0.1, 5, 5, 1, 9.82}))
%% 
% $$\theta_1 = 45^\circ, \theta_2 = 45^\circ$
Taue3 =  subs(Tau,...
    {theta1, theta1dot, theta1dotdot, theta2, theta2dot ,theta2dotdot},...
    {45/180*pi, 0, 0, 45/180*pi, 0, 0})
%%
% Taking some reasonable values, where the Center of Masses are centered
% and the links have mass 5 kg with a 1 kg payload and links are 0.3 m and
% 0.2 m long. The following torqe estimates are in Nm
eval(subs(Taue3,...
    {l1, l2, lc1, lc2, m1, m2, mL, g},...
    {0.3, 0.2, 0.15, 0.1, 5, 5, 1, 9.82}))
%% 
% *f.* Calculate inverse Jacobian
Jreduced = J(1:2, :)
Jreducedinverse = simplify(inv(Jreduced))
%% 
% *g.* Examine the stability of the inverse Jacobian
simplify(det(Jreduced))
%%
% The inverse Jacobian is undefined when the determinant of the Jacobian
% approaches zero, and this occurs as $$\theta_2$ approaches $$\pi n$ where
% n is any integer value. The physical interpretation of this effect is
% that with link 1 and link 2 aligned, either fully extended or
% overlapping, a degree of freedom is lost. Changing both $$\theta_1$ and
% $$\theta_2$ causes a motion in the same direction. Additionally, if
% either link length is zero, the arm is reduced to having a joint and
% therefore just one degree of freedom.
%% 
% *h.* Find the velocity of the end effector given that
% 
% $$L_1 = 0.3 m, L_2 = 0.2 m, \theta_1 = 30^\circ, \theta_2 = 45^\circ,
% \dot{\theta}_1 = \dot{\theta}_2 = 0.25 \frac{rad}{s}$
v = J*[theta1dot;theta2dot]
eval(subs(v,...
    {l1, l2, theta1, theta2, theta1dot, theta2dot},...
    {0.3, 0.2, 30/180*pi, 45/180*pi, 0.25, 0.25}))
%%
% The first 3 terms are in meters-per-second and correspond to linear
% velocity in (x,y,z). The last 3 terms are in radians-per-second and
% correspond to angular velocity about axes (x,y,z)
%% 
% *i.* Find the velocity of the end effector given that
% 
% $$L_1 = 0.3 m, L_2 = 0.2 m, \theta_1 = 0^\circ, \theta_2 = 0^\circ,
% \dot{\theta}_1 = \dot{\theta}_2 = 0.25 \frac{rad}{s}$
v = J*[theta1dot;theta2dot]
eval(subs(v,...
    {l1, l2, theta1, theta2, theta1dot, theta2dot},...
    {0.3, 0.2, 0/180*pi, 0/180*pi, 0.25, 0.25}))
%%
% The first 3 terms are in meters-per-second and correspond to linear
% velocity in (x,y,z). The last 3 terms are in radians-per-second and
% correspond to angular velocity about axes (x,y,z). The first term being
% zero follows intuition. The robot is in a singularity and can't move
% instantly in that direction.
%% 
% *j.* Find the Torque imposed on each joint by an external force of 10 N in the
% -x direction. The arm is in the configuration from part h.
%
% $$\tau = J^T(q)F$
%
% Where
%
% $$F = \left[ \begin{array}{c} F_x  \\ F_y \\ F_z \\ M_x \\ M_y \\ M_z
% \end{array} \right]$
syms Fx Fy Fz nx ny nz

Tau = simplify(J.' * [Fx; Fy; Fz; nx; ny; nz])
%%
% Substitute values. Torque is in units of Nm
Tauj = eval(subs(Tau,{l1,l2,theta1,theta2,Fx,Fy,Fz,nx,ny,nz}, ...
    {0.3,0.2,30/180*pi,45/180*pi,-10,0,0,0,0,0}))
%% 
% *k.* If we apply a force of 10N in the negative x-direction to the tip
% with the robot in the confiuration of Part i, what are the corresponding
% joint torques for each of the joints.
%
% Torque is again in units of Nm
syms Fx Fy Fz nx ny nz

Tau = J.' * [Fx;Fy;Fz;nx;ny;nz];

Tauk = eval(subs(Tau,{l1,l2,theta1,theta2,Fx,Fy,Fz,nx,ny,nz}, ...
    {0.3,0.2,0/180*pi,0/180*pi,-10,0,0,0,0,0}))
%%
% This agrees with intuition, a force along a link does not apply any
% torque to its joint.