function [ tf ] = TF( robot, varargin )
%TF Constructs transformation matricies for a given robot
%   This function, by default calculates the forward transform from the
%   given robot's base to it's end effector. There are numerous argument
%   functions to alter this behavior. 
%   'start_link' the to transform "from"
%   'end_link' the to transform "to"
%   'config' a given joint configuration, which will be substituted in
%   'inv' boolean. True if the inverse transformation should be returned

if isempty(robot.function_cache)
    robot.function_cache = cell(7);
end

if isempty(robot.sym_cache)
    robot.sym_cache = cell(7);
end

p = inputParser;
defaultStart = 1;
defaultEnd = robot.num_links();

addRequired(p,'robot');
addOptional(p,'start_link',defaultStart,@isnumeric);
addOptional(p,'end_link',defaultEnd,...
     @(x) isnumeric(x) && x >= 0 && x <= robot.num_links());
addParameter(p,'config',[],@isvector);
addParameter(p,'inv',false,@isvector);

parse(p,robot,varargin{:});

has_config = false;
if ~isempty(p.Results.config)
    has_config = true;
end


idx_1 = p.Results.start_link+1;
idx_2 = p.Results.end_link+1;
if p.Results.inv    
    tmp = idx_1;
    idx_1 = idx_2;
    idx_2 = tmp;
end

if isempty(robot.function_cache{p.Results.start_link+1, p.Results.end_link+1})
    tf = sym(eye(4));
    
    for link_no = p.Results.start_link : p.Results.end_link
        tf = tf * robot.get_link_tf(link_no);
    end

    if p.Results.inv
        tf = TF_Inv(tf);
    end
    
    robot.sym_cache{idx_1, idx_2} = tf;
    if has_config
        func = matlabFunction(tf, 'Vars', robot.get_joint_vars());
        robot.function_cache{idx_1, idx_2} = func;
    end
end

if has_config
    params = [p.Results.config];
    
    func = robot.function_cache{idx_1, idx_2};
    params = mat2cell(params,1,ones(1,numel(params)));
    
    tf = func( params{:} );
else 
    tf = robot.sym_cache{idx_1, idx_2};
end

