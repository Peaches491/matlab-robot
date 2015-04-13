function [L] = Lagrangian (robot)
P = 0;
K = 0;

for link_no = 1 : robot.num_links()
    masses = robot.get_masses(link_no)
    
    for mass_idx = 1 : robot.num_masses(link_no)
        massVal = masses(mass_idx, 1);
        massPos = masses(mass_idx, 2:4);
        
        disp massVal
        display(massVal)
        
        disp massPos
        massPos = [massPos, 1];
        display(massPos)
        
        translation = robot.TF('end_link', link_no) * massPos'
        Jm = simplify(robot.Jacobian('end_link', link_no, 'position', massPos'));
        Jvm = Jm(1:3,:); 
        
        syms g
        gDir = 1;
        P = P + -g * translation(gDir) * massVal;  %% gDir is determined by direction of gravity in world frame 

        qdot = robot.get_joint_var_dot(link_no);
        K = K + simplify(1/2 * qdot.' * massVal * (Jvm' * Jvm) * qdot ); 
        % qdot is the vector of the derivatives of all joint vars
    end
end
L = K - P
end

