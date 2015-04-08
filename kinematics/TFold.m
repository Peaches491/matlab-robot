function [ tf ] = TFold( robot, varargin )
%TF Constructs transformation matricies for a given robot
%   This function, by default calculates the forward transform from the
%   given robot's base to it's end effector. There are numerous argument
%   functions to alter this behavior. 
%   'start_link' the to transform "from"
%   'end_link' the to transform "to"
%   'config' a given joint configuration, which will be substituted in
%   'inv' boolean. True if the inverse transformation should be returned

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

tf = eye(4);
has_config = false;
if ~isempty(p.Results.config)
    has_config = true;
end

for link_no = p.Results.start_link : p.Results.end_link
    tf = tf * robot(link_no).T;
end


if has_config
    q = [robot(:).q];
    p.Results.config;
    tf = subs(tf, q, [p.Results.config]);
end


if p.Results.inv
    tf = TF_Inv(tf);
end
