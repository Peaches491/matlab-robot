function [eq, X, dX, U] = eom2ss_symbolic_example4(s)
% equations of motion listed in symbolic structure - use eom2ss to evaluate
%
% inputs  1 - 1 optional
% s       flag to inidcate symbolic (1) or numerical (0)    - class integer
%
% outputs 4
% eq      system of equations in a structure                - class struct
% x       states in cell array                              - class cell
% dx      deravitive of states in cell array                - class cell
% U       control inputs to system in cell arrray           - class cell
%
% michael arant - jan 12, 2013

if nargin == 0; s = 0; end

% declare the states and inputs
syms x1 x2 dx1 dx2 u1 u2 ddx1 ddx2 dddx1 ddddx1
X = {'x1' 'x2' 'dx1' 'dx2' 'ddx1' 'dddx1'};
dX = {'dx1' 'dx2' 'ddx1' 'ddx2' 'dddx1' 'ddddx1'};
U = {'u1' 'u2'};

% symbolic system or numerical?
if s
	syms a b c d e f g h i j k m n
else
	a = 1; b = 2; c = 3; c = 4; d = 5; e = 6; f = 7; g = 8; h = 9;
	i = 10; j = 11; k = 12; m = 13; n = 14;
end


% note example here has two variables with two deravitives so 
% two identity equations are needed to develop the state space (last two) 
eq.e1 = [char(a*ddx1 + k*ddddx1) ' = ' char(c*x1 + d*x2 + i*dx1 - j*dx2 + e*u1)];
eq.e2 = [char(f*ddx2 + n*dddx1) ' = ' char(g*x2 + b*dx1 - h*u2 + c*dddx1 + n*ddx1)];
eq.e3 = 'dx1 = dx1';
eq.e4 = 'dx2 = dx2';
eq.e5 = 'ddx1 = ddx1';
eq.e6 = 'dddx1 = dddx1';

