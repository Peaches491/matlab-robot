function [ tf ] = TF( robot, varargin )
%TF Constructs transformation matricies for a given robot
%   This function, by default calculates the forward transform from the
%   given robot's base to it's end effector. There are numerous argument
%   functions to alter this behavior. 
%   'start_link' the to transform "from"
%   'end_link' the to transform "to"
%   'config' a given joint configuration, which will be substituted in
%   'inv' boolean. True if the inverse transformation should be returned

persistent function_cache;
persistent sym_cache;

if isempty(function_cache)
    disp 'Initializing Function Cache!'
    function_cache = cell(7);
end

if isempty(sym_cache)
    disp 'Initializing Sym Cache!'
    sym_cache = cell(7);
end

p = inputParser;
defaultStart = 1;
defaultEnd = size(robot, 2);

addRequired(p,'robot');
addOptional(p,'start_link',defaultStart,@isnumeric);
addOptional(p,'end_link',defaultEnd,...
     @(x) isnumeric(x) && x >= 0 && x <= size(robot, 2));
addParameter(p,'config',[],@isvector);
addParameter(p,'inv',false,@isvector);

parse(p,robot,varargin{:});

has_config = false;
if ~isempty(p.Results.config)
    has_config = true;
end


if isempty(function_cache{p.Results.start_link+1, p.Results.end_link+1})
    tf = sym(eye(4));
    
    for link_no = p.Results.start_link : p.Results.end_link
        tf = tf * robot(link_no).T;
    end

    if p.Results.inv
        tf = TF_Inv(tf);
    end
    
    func = matlabFunction(tf, 'Vars', [robot(:).q]);
    
    if p.Results.inv
        function_cache{p.Results.end_link+1, p.Results.start_link+1} = func;
        sym_cache{p.Results.end_link+1, p.Results.start_link+1} = tf;
    else 
        function_cache{p.Results.start_link+1, p.Results.end_link+1} = func;
        sym_cache{p.Results.start_link+1, p.Results.end_link+1} = tf;
    end
end


if has_config
    params = [p.Results.config];
    
    func = @isempty;
    if p.Results.inv
        func = function_cache{p.Results.end_link+1, p.Results.start_link+1};
    else 
        func = function_cache{p.Results.start_link+1, p.Results.end_link+1};
    end
    tf = func(params(1), params(2), params(3), params(4), params(5), params(6));
else 
    tf = sym_cache{p.Results.start_link+1, p.Results.end_link+1};
end

