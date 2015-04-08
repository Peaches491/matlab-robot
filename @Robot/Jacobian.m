function [ J ] = Jacobian(robot, varargin)

p = inputParser;
defaultEnd = robot.num_links();
defaultPos = [0 0 0 1]';

addRequired(p,'robot');
addOptional(p,'position',defaultPos,@isvector);
addOptional(p,'end_link',defaultEnd,...
     @(x) isnumeric(x) && x >= 0 && x <= size(robot, 2));
parse(p,robot,varargin{:});

translation = robot.TF('end_link', p.Results.end_link) * p.Results.position; 
translation = translation(1:3);

Jv = [];
Jw = [];

for link_no = 1:robot.num_links()
    Jv = [Jv, diff(translation, robot.get_joint_var(link_no))];
    linkTF = robot.TF('end_link', link_no);
    Jw = [Jw, linkTF(1:3,3)];
end

J = [Jv; Jw];
