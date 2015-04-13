function [L] = Lagrangian (robot)
P = 0;
K = 0;

for link_no = 1 : robot.num_links()
    masses = robot.get_masses(link_no);
    
    for mass_idx = 1 : robot.num_masses(link_no)
        massVal = masses(mass_idx, 1);
        massPos = masses(mass_idx, 2:4);
        massPos = [massPos, 1];
        
        translation = robot.TF('end_link', link_no) * massPos';
        Jm = simplify(robot.Jacobian('end_link', link_no, 'position', massPos'));
        Jvm = Jm(1:3,:); 
        
        syms g
        gDir = 1;
        P = P + -g * translation(gDir) * massVal  %% gDir is determined by direction of gravity in world frame 

        qdots = robot.get_joint_vars_dot().';
        
        %K = K + simplify(1/2 * qdot.' * massVal * (Jvm' * Jvm) * qdot )
        K = K + (1/2 * qdots.' * massVal * (Jvm' * Jvm) * qdots )
        % qdot is the vector of the derivatives of all joint vars
    end
end
L = K - P
end

