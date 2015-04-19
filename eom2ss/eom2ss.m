function [M, K, I, A, B] = eom2ss(eq,x,dx,U)
% generate state matrix from equations of motion given states and inputs
% function [M, K, I, A, B] = eom_gen(fn,dr,)
%
% inputs  4 
% eq      equations of motion (n eqn in structure)      - class structure
% x       system states (1xn cell array)                - class cell
% dx      deravitive of system states (1xn cell array)  - class cell
% U       system inputs                                 - class cell
%
% outputs 5
% M       mass matrix (symbolic or numeric) (nxn)       - class sym or real
% K       stiffnes matrix (symbolic or numeric) (nxn)   - class sym or real
% I       Input matrix (symbolic or numeric) (nxm)      - class sym or real
% A       state A matrix (numerical case only) (nxn)    - class real
% B       input matrix (numerical case only) (nxm)      - class real
%
% michael arant - jan 10, 2013
%
% solves system of equations and generates state space matrix form
% x_dot * M = x * K + I * u
% where:
% A = M \ K and B = M \ I
% x_dot = A * x + B * u
%
% Note:  A and B are NOT computed for symbolic cases as systems over ~ 10 states
% will crash memory (too large).

%% file input check
warning('off','symbolic:mupadmex:MuPADTextWarning');

if nargin < 4; error('Need equations, states, state deravitives, and inputs'); end

%% consistancy check

% collect the field information
eqn = fieldnames(eq); n = numel(eqn);
if numel(x) ~= n; error('Number of states must match number of equations'); end
if numel(dx) ~= n; error('Number of derivatives must match number of equations'); end

% null space values
nulls = [1:32 127:160];


%% parse the eom into state matrix

% program loops each equation searching for state variables which are
% captured and used to populate the M (mass) K (stiffness) and I (input)
% matricies

% lookfor second order variables - extract and record first order only
% this prevents algrothim from deleting the first deravitive from the 
% system completely
var = find(~ismember(dx,x));

% option to print system equations - just uncomment
% for ii = 1:n
% 	disp(' '); pretty(sym(char(eq.(fn{ii}))));
% end
% disp(' '); disp(' ');



%% convert to mass stiffness form
% size system
n = numel(x); m = numel(U); fn = fieldnames(eq);

ssm = sym(0); wb = waitbar(0,'Evaluate Equations');
M = 0 * ssm * zeros(n,n); K = M; I = 0 * ssm * zeros(n,numel(U));
for ii = 1:n
	for jj = 1:n
		waitbar((((ii-1)*n)+jj)/(n*n),wb); drawnow
		% evaluate Mass structure
		% get the equation
		temp = eq.(fn{ii}); temp = char(temp);
		% toss '==' - this is a change in matlabs symbolic notation....
		de = findstr('==',temp);
		if ~isempty(de); temp(de) = []; end

		% evaluate mass structure
		% check for equality statement
		tempe = findstr('=',(temp));
		if strcmp(strtrim(temp(1:tempe-1)),strtrim(temp(tempe+1:end))) && ii == jj
			% equality statemet - pass through
			% find location
			M(ii,find(ismember(dx,sym(strtrim(temp(1:tempe-1)))))) = 1;
			K(ii,find(ismember(x,sym(strtrim(temp(1:tempe-1)))))) = 1; 
		else
			% isolate the deravitives
			for kk = 1:n
				if ~strcmp(dx(jj),dx(kk))
					if max(findstr(char(temp),char(dx(kk))))
						temp = subs(temp,dx(kk),0);						
						if ~strcmp(class(temp),'sym'); temp = sym(temp); end
					end
				end
			end
			% isolate the states
			for kk = 1:n
				if max(findstr(char(temp),char(x(kk))))
					temp = subs(temp,x(kk),0);
					if ~strcmp(class(temp),'sym'); temp = sym(temp); end
				end
			end
			% isolate the inputs
			for kk = 1:m
				if max(findstr(char(temp),char(U(kk))))
					temp = subs(temp,U(kk),0);
					if ~strcmp(class(temp),'sym'); temp = sym(temp); end
				end
			end

			% populate the mass matrix
			if ~isempty(findstr(char(dx(jj)),char(temp)))
				% check for extranious parameter (forgotten input or other)
				tempd = subs(temp,dx(jj),'0');
				tempd = strrep(char(tempd),'=','');
				tempd = strrep(char(tempd),'0','');
				tempd(ismember(double(tempd),nulls)) = [];
				if ~isempty(deblank(tempd))
 					error('Forgoten input or unknown parameter: %s', ...
 						tempd)
				end
				% sub out parameter
				temp = subs(temp,dx(jj),'1');
				% if valid coefficient - populate
				if isa(temp,'sym')
					% convert to charater
					temp = char(temp); temp = strrep(temp,' ','');
					% find = - rempve == from symbolic format change in matlab
					de = findstr('==',temp); if ~isempty(de); temp(de) = []; end
					esign = findstr('=',char(temp));
					% split left right
					templ = temp(1:esign-1); tempr = temp(esign+1:end);
					templ = strtrim(templ); tempr = strtrim(tempr);
					if ~isempty(templ); M(ii,jj) = M(ii,jj) + templ; end
					if ~isempty(tempr); M(ii,jj) = M(ii,jj) - tempr; end
				end
			end


			% evaluate stiffness structure
			% get the equation again
			temp = eq.(fn{ii});
			% isolate the variables
			for kk = 1:n
				if ~strcmp(x(jj),x(kk))
					if max(findstr(char(temp),char(x(kk))))
						temp = subs(temp,x(kk),0);
						if ~strcmp(class(temp),'sym'); temp = sym(temp); end
					end
				end
			end
			% isolate deravitives but do not remove if second deravitive exists
			for kk = var
				if  max(findstr(char(temp),char(dx(kk))))
					temp = subs(temp,dx(kk),0);
					if ~strcmp(class(temp),'sym'); temp = sym(temp); end
				end
			end
			% isolate inputs
			for kk = 1:m
				if  max(findstr(char(temp),char(U(kk))))
					temp = subs(temp,U(kk),0);
					if ~strcmp(class(temp),'sym'); temp = sym(temp); end
				end
			end
			
			% populate stiffness matrix
			if ~isempty(findstr(char(x(jj)),char(temp)))
			% check for extranious parameter (forgotten input or other)
			tempd = subs(temp,x(jj),'0');
			tempd = strrep(char(tempd),'=','');
			tempd = strrep(char(tempd),'0','');
			tempd(ismember(double(tempd),nulls)) = [];
			if ~isempty(deblank(tempd))
 				error('Forgoten input or unknown parameter: %s', ...
 						tempd)
			end
			% sub out parameter
				temp = subs(temp,x(jj),'1');
				if isa(temp,'sym')
					% convert to charater
					temp = char(temp); temp = strrep(temp,' ','');
					% find =
					de = findstr('==',temp); if ~isempty(de); temp(de) = []; end
					esign = findstr('=',char(temp));
					% split left right
					templ = temp(1:esign-1); tempr = temp(esign+1:end);
					templ = strtrim(templ); tempr = strtrim(tempr);
					if ~isempty(templ); K(ii,jj) = K(ii,jj) - templ; end
					if ~isempty(tempr); K(ii,jj) = K(ii,jj) + tempr; end
				end
			end
		end
	end
	% check for zero M row (no final state deravitive in equation - 
	% invalid form of equations)
	Mr = M(ii,:);
	if isa(Mr,'sym')
		% convert sym to char to number - drop 0 (48) , (44) and space (32)
		Mr = double(char(Mr)); Mr([1:9 end-2:end]) = [];
		Mr(Mr == 44) = []; Mr(Mr == 32) = []; Mr(Mr == 48) = [];
	else
		Mr(~Mr) = [];
	end
%   if isempty(Mr)
%		close(wb)
%		error('Equation %g does not have a final state deravitive',ii); 
%	end

	% evaluate input structure
	% get the equation
	temp = eq.(fn{ii});
	% isolate the deravitives
	for kk = 1:n
		if max(findstr(char(temp),char(dx(kk))))
			temp = subs(temp,dx(kk),0);
			if ~strcmp(class(temp),'sym'); temp = sym(temp); end
		end
	end
	% isolate the states
	for kk = 1:n
		if  max(findstr(char(temp),char(x(kk))))
			temp = subs(temp,x(kk),0);
			if ~strcmp(class(temp),'sym'); temp = sym(temp); end
		end
	end
	% build inputs
	% check for posible symbolic conversions - make sure parameter exists...
	if ~strcmp(strrep(char(temp),' ',''),'0=0') && ...
			~strcmp(strrep(char(temp),' ',''),'0.0=0.0') && ...
			~strcmp(strrep(char(temp),' ',''),'0=0.0') && ...
			~strcmp(strrep(char(temp),' ',''),'0.0=0')
		junk = temp;
		for kk = 1:m
			temp = junk;
			% remove inputs other than current target input
			for ll = 1:m
				if kk ~= ll && ~strcmp(strrep(char(temp),' ',''),'0=0') && ...
						~isempty(findstr(char(temp),char(U(ll))))
					% drop other inputs
					temp = subs(temp,U(ll),0);
					if ~strcmp(class(temp),'sym'); temp = sym(temp); end
				end
			end
			% record ionput
			if ~strcmp(strrep(char(temp),' ',''),'0=0')
			% check for extranious parameter (forgotten input or other)
			tempd = subs(temp,U(kk),'0');
			tempd = strrep(char(tempd),'=','');
			tempd = strrep(char(tempd),'0','');
			tempd(ismember(double(tempd),nulls)) = [];
%			if ~isempty(deblank(tempd))
%					error('Forgoten input or unknown parameter: %s', ...
%						tempd)
%			end
			% sub out parameter
				temp = subs(temp,U(kk),'1');
				if isa(temp,'sym')
					% convert to charater
					temp = char(temp); temp = strrep(temp,' ','');
					% find =
					de = findstr('==',temp); if ~isempty(de); temp(de) = []; end
					esign = findstr('=',char(temp));
					% split left right
					templ = temp(1:esign-1); tempr = temp(esign+1:end);
					templ = strtrim(templ); tempr = strtrim(tempr);
					if ~isempty(templ); I(ii,kk) = I(ii,kk) - templ; end
					if ~isempty(tempr); I(ii,kk) = I(ii,kk) + tempr; end
				end
			end
		end
	end
end
close(wb); drawnow

%% if system is numerical and A and B requested, calculate
try; M = double(M); end
try; K = double(K); end
try; I = double(I); end

if isa(M,'sym')
	M = simplify(M);
	K = simplify(K);
	I = simplify(I);
else
	A = M \ K;
	B = M \ I;
end

return

