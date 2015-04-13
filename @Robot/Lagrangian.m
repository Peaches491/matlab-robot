function [L] = Lagrangian (robot)
P = 0;
K = 0;
% for Link in robot
% for mass on link 
     translation = robot.TF('end_link', link)* massPos
     Jm = simplify(robot.Jacobian('end_link', link, 'position', massPos));
     Jvm = Jm(1:3,:);
    % where massPos is the position of the mass in the link frame 
 
    P = P + -g * translation(gDir) * massVal;  %% gDir is determined by direction of gravity in world frame 
   
    K = K + simplify(1/2 * qdot.' * ...
    massVal * (Jvm1.' * Jvm)...
    * qdot ); 
    % qdot is the vector of the derivatives of all joint vars
    % where massVal is the value of the mass 
    
    L = K - P
end

