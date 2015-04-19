function [sq, X, dX, U, eq, p] = eom2ss_symbolic_example5(s)
% equations of motion listed in symbolic structure - use eom2ss to evaluate
%
% inputs  1 - 1 optional
% s       flag to inidcate symbolic (1) or numerical (0)    - class integer
%
% outputs 4
% sq      system of equations in a structure                - class struct
% x       states in cell array                              - class cell
% dx      deravitive of states in cell array                - class cell
% U       control inputs to system in cell arrray           - class cell
%
% michael arant - jan 12, 2013

if nargin == 0; s = 0; end

% declare the states and inputs
syms d0B1 d1P1 d1T1 d0T1 d0B2 d1P2 d1T2 d0T2 d0B3 d1P3 d1T3 d0T3 
syms d0B4 d1P4 d1T4 d0T4 d0B5 d1P5 d1T5 d0T5 d0B6 d1P6 d1T6 d0T6
syms d1B1 d2P1 d2T1 d1T1 d1B2 d2P2 d2T2 d1T2 d1B3 d2P3 d2T3 d1T3 
syms d1B4 d2P4 d2T4 d1T4 d1B5 d2P5 d2T5 d1T5 d1B6 d2P6 d2T6 d1T6
syms Del g Mef Mer Me2 Me3 Me4 Me5 Me6 F12 F23 F34 F45 F56

% states
X = {'d0B1' 'd1P1' 'd1T1' 'd0T1' 'd0B2' 'd1P2' 'd1T2' 'd0T2' 'd0B3' 'd1P3' 'd1T3' 'd0T3' ...
	'd0B4' 'd1P4' 'd1T4' 'd0T4' 'd0B5' 'd1P5' 'd1T5' 'd0T5' 'd0B6' 'd1P6' 'd1T6' 'd0T6'};
dX = {'d1B1' 'd2P1' 'd2T1' 'd1T1' 'd1B2' 'd2P2' 'd2T2' 'd1T2' 'd1B3' 'd2P3' 'd2T3' 'd1T3' ...
	'd1B4' 'd2P4' 'd2T4' 'd1T4' 'd1B5' 'd2P5' 'd2T5' 'd1T5' 'd1B6' 'd2P6' 'd2T6' 'd1T6'};
U = {'Del' 'Mef' 'Mer' 'Me2' 'Me3' 'Me4' 'Me5' 'Me6'};

% symbolic system or numerical?
if s
	syms U1 U2 U3 U3 U4 U5 U6 m1 m2 m3 m4 m5 m6 Z1 Z2 Z3 Z4 Z5 Z6
	syms Ixx1 Ixx2 Ixx3 Ixx4 Ixx5 Ixx6 Izz1 Izz2 Izz3 Izz4 Izz5 Izz6
	syms Ixz1 Ixz2 Ixz3 Ixz4 Ixz5 Ixz6 Caf Car Ca2 Ca3 Ca4 Ca5 Ca6
	syms Kf Kr K2 K3 K4 K5 K6 Lf Lr L2 L3 L4 L5 L6
	syms a1 a2 a3 a4 a5 a6 b1 b2 b3 b4 b5 b6 c1 c2 c3 c4 c5 c6
	syms d1 d2 d3 d4 d5 d6 f1 f2 f3 f4 f5 f6 g1 g2 g3 g4 g5 g6
	syms F12 F23 F34 F45 F56
else
	g = 9.81; U1 = 10;
	U2 = U1; U3 = U1; U4 = U1; U5 = U1; U6 = U1;
	m1 = 4457; Izz1 = 34823; Ixx1 = 2287; Ixz1 = 1626;
	hr1 = 0.6; fw1 = 2.750; Lf = 50000; Lr = Lf; %500000
	hr2 = 0.9; g2 = 0; L2 = Lf;
	m3 = 500; Ixx3 = 100; Izz3 = 125; Ixz3 = 0;
	hr3 = 0.9; g3 = 0;
	L3 = Lf; fw3 = 1.86;
	WB1 = 3.5; WB2 = 6.7; WB3 = 2.1;
	OL2 = 7.5; spcgh1 = 1.173-hr1; spcgh3 = 1.15 - hr3;
	m2 = 3000; Ixx2 = 5964; Izz2 = 27000; Ixz2 = 600;
	Caf = 221000; Car = 400000; Ca2 = 220000; Ca3 = 225000;
	Caf = 205000; Car = 350000; Ca2 = 276000; Ca3 = 200000;
	Kf = 322400; Kr = 550000; K2 = 676800; K3 = 300000;
	Kf = 350000; Kr = 550000; K2 = 676800; K3 = 300000;
	a1 = 1.5270; a2 = 4.4870; a3 = 1.7244;
	spcgh2 = 1.935-hr2;
	OL4 = OL2; OL6 = OL2; fw5 = fw3;
	a4 = a2; a5 = a3; a6 = a2; WB4 = WB2; WB5 = WB3; WB6 = WB2;
	Z1 = spcgh1 + (hr1/spcgh1) * (spcgh1+hr1)/8;
	Z2 = spcgh2 + (hr2/spcgh2) * (spcgh2+hr2)/8;
	Z3 = spcgh3 + (hr3/spcgh3) * (spcgh3+hr3)/8;
	K4 = K2; L4 = L2; Ca4 = Ca2; m4 = m2;
	K5 = K3; L5 = L3; Ca5 = Ca3; m5 = m3;
	K6 = K2; L6 = L2; Ca6 = Ca2; m6 = m2;
	Ixx4 = Ixx2; Ixz4 = Ixz2; Izz4 = Izz2; 
	Ixx5 = Ixx3; Ixz5 = Ixz3; Izz5 = Izz3;
	Ixx6 = Ixx2; Ixz6 = Ixz2; Izz6 = Izz2;
	f1 = 1.1 - hr1; f2 = 1.1 - hr2; f3 = 1.1 - hr3;
	f4 = f2; f5 = f3; f6 = f4; g4 = g2; g5 = g3; g6 = g2;
	Z4 = Z2; Z5 = Z3; Z6 = Z2;
	b1 = WB1 - a1; c1 = fw1 - a1;
	b2 = WB2 - a2; c2 = a2; d2 = OL2 - a2;
	b3 = WB3 - a3; c3 = fw3 - a3; d3 = a3;
	b4 = WB4 - a4; c4 = a4; d4 = OL4 - a4;
	b5 = WB3 - a5; c5 = fw5 - a5; d5 = a5;
	b6 = WB6 - a6; c6 = a6; d6 = OL6 - a6;
end

% zero negable terms
g2 = 0; g3 = 0; g4 = 0; g5 = 0; k12 = 0; k23 = 0; k34 = 0; k45 = 0; k56 = 0;


p.e1.l = char(m1 * U1 *(d1B1 + d1P1) - m1 * Z1 * d2T1);
p.e1.r = char((b1*Car -a1*Caf)*d1P1/U1 -(Caf+Car)*d0B1 - F12 + Caf*Del);
p.e2.l = char(Izz1 * d2P1 - Ixz1 * d2T1 + m1*Z1*U1*d1T1);
p.e2.r = char(-(a1*Caf-b1*Car)*d0B1 - (a1^2 * Caf + b1^2 * Car) * d1P1/U1 + c1 * F12 + a1*Caf*Del + Mef + Mer);
p.e3.l = char(d2T1*(m1*Z1^2 + Ixx1) - Ixz1*d2P1);
p.e3.r = char(U1*Z1*m1*(d1B1 + d1P1) - d0T1*(Kf + Kr) - d1T1*(Lf + Lr) + F12*f1 + d0T1*Z1*g*m1);
p.e4.l = 'd1T1';
p.e4.r = p.e4.l;
p.e5.l = char(m2*U2*(d1B2+d1P2) - m2*Z2*d2T2);
p.e5.r = char(F12-F23 + b2*Ca2*d1P2/U2 - Ca2*d0B2);
p.e6.l = char(Izz2*d2P2 - Ixz2 * d2T2 + m2*Z2*U2*d1T2);
p.e6.r = char(c2*F12+d2*F23 + b2*Ca2*d0B2 - b2^2*Ca2*d1P2/U2 + Me2);
p.e7.l = char((Ixx2 + m2*Z2^2)*d2T2 - Ixz2*d2P2);
p.e7.r = char(m2*Z2*(g*d0T2 + U2*(d1B2 + d1P2)) - K2*d0T2 - L2*d1T2 - f2*F12 +g2*F23); %%
p.e8.l = 'd1T2';
p.e8.r = p.e8.l;
p.e9.l = char(d1B2);
p.e9.r = char(d1B1 - c1*d2P1/U1 - c2*d2P2/U2 + d1P1 - d1P2);
p.e10.l = char(m3 *U3*(d1B3+d1P3) - m3*Z3*d2T3);
p.e10.r = char(-F34+F23+Ca3*b3*d1P3/U3-Ca3*d0B3);
p.e11.l = char(Izz3* d2P3-Ixz3* d2T3 + m3*Z3*U3*d1T3);
p.e11.r = char(d3*F23+c3*F34+b3*Ca3*d0B3-Ca3*b3^2*d1P3/U3 + Me3);
p.e12.l = char((Ixx3 + m3 * Z3 ^ 2) * d2T3 - Ixz3 * d2P3);
p.e12.r = char(m3 * Z3 * g * d0T3 + m3 * Z3 * U3 * (d1B3 + d1P3) - K3 * d0T3 - L3 * d1T3 + f3 * F34 -g3*F23);
p.e13.l = char(d1T3);
p.e13.r = char(d1T3);
p.e14.l = char(d1B3);
p.e14.r = char(d1B2 - c2 * d2P2 / U2 - c3 * d2P3 / U3 + d1P2 - d1P3);
p.e15.l = char(m4 * U4 * (d1B4 + d1P4) - m4 * Z4 * d2T4);
p.e15.r = char(-Ca4 * d0B4 + b4 * Ca4 * d1P4 / U4 + F34 - F45);
p.e16.l = char(Izz4 * d2P4 - Ixz4 * d2T4 + m4*Z4*U4*d1T4);
p.e16.r = char(b4 * Ca4 * d0B4 - b4 ^ 2 * Ca4 * d1P4 / U4 + c4 * F34 +d4*F45+ Me4);
p.e17.l = char((Ixx4 + m4 * Z4 ^ 2) * d2T4 - Ixz4 * d2P4);
p.e17.r = char(m4 * Z4 * (g * d0T4 + U4 * (d1B4 + d1P4)) - K4 * d0T4 - L4 * d1T4 - f4 * F34+g4*F45);
p.e18.l = char(d1T4);
p.e18.r = char(d1T4);
p.e19.l = char(d1B4);
p.e19.r = char(d1B3 - c3 * d2P3 / U3 - c4 * d2P4 / U4 + d1P3 - d1P4);
p.e20.l = char(m5 * U5 * (d1B5 + d1P5) - m5 * Z5 * d2T5);
p.e20.r = char(F45-F56 + Ca5 * b5 * d1P5 / U5 - Ca5 * d0B5);
p.e21.l = char(Izz5 * d2P5 - Ixz5 * d2T5 + m5*Z5*U5*d1T5);
p.e21.r = char(d5*F45+c5 * F56 + b5 * Ca5 * d0B5 - Ca5 * b5 ^ 2 * d1P5 / U5 + Me5);
p.e22.l = char((Ixx5 + m5 * Z5 ^ 2) * d2T5 - Ixz5 * d2P5);
p.e22.r = char(m5 * Z5 * g * d0T5 + m5 * Z5 * U5 * (d1B5 + d1P5) - K5 * d0T5 - L5 * d1T5 + f5 * F56 - g5*F45);
p.e23.l = char(d1T5);
p.e23.r = char(d1T5);
p.e24.l = char(d1B5);
p.e24.r = char(d1B4 - c4 * d2P4 / U4 - c5 * d2P5 / U5 + d1P4 - d1P5);
p.e25.l = char(m6 * U6 * (d1B6 + d1P6) - m6 * Z6 * d2T6);
p.e25.r = char(-Ca6 * d0B6 + b6 * Ca6 * d1P6 / U6 + F56);
p.e26.l = char(Izz6 * d2P6 - Ixz6 * d2T6 + m6*Z6*U6*d1T6);
p.e26.r = char(b6 * Ca6 * d0B6 - b6 ^ 2 * Ca6 * d1P6 / U6 + c6 * F56 + Me6);
p.e27.l = char((Ixx6 + m6 * Z6 ^ 2) * d2T6 - Ixz6 * d2P6);
p.e27.r = char(m6 * Z6 * (g * d0T6 + U6 * (d1B6 + d1P6)) - K6 * d0T6 - L6 * d1T6 - f6 * F56);
p.e28.l = char(d1T6);
p.e28.r = char(d1T6);
p.e29.l = char(d1B6);
p.e29.r = char(d1B5 - c5 * d2P5 / U5 - c6 * d2P6 / U6 + d1P5 - d1P6);


% build equations
for ii = 1:29
	eq.(['e' num2str(ii)]) = [char(p.(['e' num2str(ii)]).l) ' = ' char(p.(['e' num2str(ii)]).r)];
end


% solve forces - defines internal forces
% F = solve(eq.e5,eq.e9,eq.e13,eq.e17,eq.e21,F12,F23,F34,F45,F56);
F = solve(eq.e6,eq.e11,eq.e16,eq.e21,eq.e26,F12,F23,F34,F45,F56);

% cancel internal forces
fn = fieldnames(eq); Fn = fieldnames(F);
for ii = 1:numel(fn)
	for jj = 1:numel(Fn)
		eq.(fn{ii}) = strrep(char(eq.(fn{ii})),char(Fn{jj}),char(F.(Fn{jj})));
	end
% 	fprintf('%s: ',fn{ii}); disp(sq.(fn{ii}));
end


% replicate eom for solver
sq.e1 = eq.e1;
sq.e2 = eq.e2;
sq.e3 = eq.e3;
sq.e4 = eq.e4;
sq.e5 = eq.e5;
sq.e6 = eq.e7;
sq.e7 = eq.e9;
sq.e8 = eq.e8;
sq.e9 = eq.e10;
sq.e10 = eq.e12;
sq.e11 = eq.e14;
sq.e12 = eq.e13;
sq.e13 = eq.e15;
sq.e14 = eq.e17;
sq.e15 = eq.e19;
sq.e16 = eq.e18;
sq.e17 = eq.e20;
sq.e18 = eq.e22;
sq.e19 = eq.e24;
sq.e20 = eq.e23;
sq.e21 = eq.e25;
sq.e22 = eq.e27;
sq.e23 = eq.e29;
sq.e24 = eq.e28;
