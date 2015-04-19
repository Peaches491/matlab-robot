clear all; close all

%% load the first example - simple baseic state space
% load eom
[eq, X, dX, U] = eom2ss_symbolic_example1(1);
disp('States:'); disp(X); disp('Inputs'); disp(U)

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)
uiwait(msgbox('Example 1 in symbolic form'))


[eq, X, dX, U] = eom2ss_symbolic_example1(0);

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)

if rank(ctrb(A,B)) == rank(A); disp('Controlable system'); end
uiwait(msgbox('Example 1 in numerical form'))

%% load the second example - coupled system
fprintf('\n\n\n\n');
% load eom
[eq, X, dX, U] = eom2ss_symbolic_example2(1);
disp('States:'); disp(X); disp('Inputs'); disp(U)

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)
uiwait(msgbox('Example 2 in symbolic form'))

[eq, X, dX, U] = eom2ss_symbolic_example2(0);

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)

if rank(ctrb(A,B)) == rank(A); disp('Controlable system'); end
uiwait(msgbox('Example 2 in numerical form'))


%% load the third eqample - coupled system with third state
fprintf('\n\n\n\n');
% load eom
[eq, X, dX, U] = eom2ss_symbolic_example3(1);
disp('States:'); disp(X); disp('Inputs'); disp(U)

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)
uiwait(msgbox('Example 3 in symbolic form'))

[eq, X, dX, U] = eom2ss_symbolic_example3(0);

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)

if rank(ctrb(A,B)) == rank(A); disp('Controlable system'); end
uiwait(msgbox('Example 3 in numerical form'))

%% load the fourth eqample - second deravitive example
fprintf('\n\n\n\n');
% load eom
[eq, X, dX, U] = eom2ss_symbolic_example4(1);
disp('States:'); disp(X); disp('Inputs'); disp(U)

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)
uiwait(msgbox('Example 4 in symbolic form'))

[eq, X, dX, U] = eom2ss_symbolic_example4(0);

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)

if rank(ctrb(A,B)) == rank(A); disp('Controlable system'); end
uiwait(msgbox('Example 4 in numerical form'))

%% load the fifth eqample - forth deravitive example
fprintf('\n\n\n\n');
% load eom
[eq, X, dX, U] = eom2ss_symbolic_example5(1);
disp('States:'); disp(X); disp('Inputs'); disp(U)

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)
uiwait(msgbox('Example 5 in symbolic form'))

[eq, X, dX, U] = eom2ss_symbolic_example5(0);

fn = fieldnames(eq);
for ii = 1:numel(fn)
	pretty(sym(eq.(fn{ii})))
end

% solve eom
[M, K, I] = eom2ss(eq,X,dX,U);
disp('M'); disp(M)
disp('K'); disp(K)
disp('I'); disp(I)
disp('A'); A = M\K; disp(A)
disp('B'); B = M\I; disp(B)

if rank(ctrb(A,B)) == rank(A); disp('Controlable system'); end
uiwait(msgbox('Example 5 in numerical form'))


%% load the sixth eqample - 24 state coupled system
fprintf('\n\n\n\n');
k = menu(char('Next example is a 24 state coupled system.', ...
	'It takes a very long time to run'),'Exit','Continue') -1;
if k
	k = menu('Evaluate symbolic case','No','Yes') -1;
	if k
		% load eom
		[eq, X, dX, U] = eom2ss_symbolic_example6(1);
		disp('States:'); disp(X); disp('Inputs'); disp(U)

		fn = fieldnames(eq);
		for ii = 1:numel(fn)
			pretty(sym(eq.(fn{ii})))
		end

		% solve eom
		[M, K, I] = eom2ss(eq,X,dX,U);
		disp('M'); disp(M)
		disp('K'); disp(K)
		disp('I'); disp(I)
		uiwait(msgbox('Example 6 in symbolic form'))

	end
	
	k = menu('Evaluate numerical case','No','Yes') -1;
	if k
		[eq, X, dX, U] = eom2ss_symbolic_example6(0);
		disp('States:'); disp(X); disp('Inputs'); disp(U)

		fn = fieldnames(eq);
		for ii = 1:numel(fn)
			pretty(sym(eq.(fn{ii})))
		end

		% solve eom
		[M, K, I] = eom2ss(eq,X,dX,U);
		disp('M'); disp(M)
		disp('K'); disp(K)
		disp('I'); disp(I)
		disp('A'); A = M\K; disp(A)
		disp('B'); B = M\I; disp(B)
		uiwait(msgbox('Example 6 in numerical form'))
	end
end
