function [J] = Jacobian(robot, varargin)

p = inputParser;
defaultEnd = size(robot, 2);
defaultPos = [0 0 0 1]';

addRequired(p,'robot');
addOptional(p,'position',defaultPos,@isvector);
addOptional(p,'end_link',defaultEnd,...
     @(x) isnumeric(x) && x >= 0 && x <= size(robot, 2));
parse(p,robot,varargin{:});

translation = TFold(robot, 'end_link', p.Results.end_link) * p.Results.position; 
 
Jv = [];
Jw = [];

for link_no = 1 :size(robot, 2);
    Jv = [Jv, diff(translation,robot(link_no).q)];  
    linkTF = TFold(robot, 'end_link', link_no);
    Jw = [Jw, linkTF(1:3,3)];
end

J = [Jv; Jw];
