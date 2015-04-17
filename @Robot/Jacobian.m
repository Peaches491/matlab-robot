function [ J ] = Jacobian(robot, varargin)

p = inputParser;
defaultEnd = robot.num_links();
defaultPos = [0 0 0 1]';

addRequired(p,'robot');
addOptional(p,'position',defaultPos,@isvector);
addOptional(p,'end_link',defaultEnd,...
     @(x) isnumeric(x) && x >= 0 && x <= robot.num_links());
parse(p,robot,varargin{:});

translation = robot.TF('end_link', p.Results.end_link) * p.Results.position; 
translation = translation(1:3);

Jv = [];
Jw = [];

for link_no = 1:robot.num_links()
    if robot.has_joint_var(link_no, 0, false)
        Jv = [Jv, diff(translation, robot.get_joint_var(link_no, 0, false))];
    else
        Jv = [Jv, [0;0;0]];
    end
    linkTF = robot.TF('end_link', link_no);
    Jw = [Jw, linkTF(1:3,3)];
end

J = [Jv; Jw];
