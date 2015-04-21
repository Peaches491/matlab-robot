function [L] = Lagrangian (robot)
P = 0;
K = 0;

for link_no = 1 : robot.num_links()
    masses = robot.get_masses(link_no);
    
    for mass_idx = 1 : robot.num_masses(link_no)
        massVal = masses(mass_idx, 1);
        
        %Mike, what does this vector do?
        massPos = masses(mass_idx, 2:4);
        massPos = [massPos, 1];
        
        % You orignally multiplied by 'massPos' here
        translation = robot.TF('end_link', link_no); % * massPos'
        Jm = simplify(robot.Jacobian('end_link', link_no, 'position', massPos'));
        Jvm = Jm(1:3,:); 
        
        % But, I think you need to pull out the translation section from
        % the TF first, As I did here. 
        translation = translation(1:3, 4);
        
        % Then you can do whatever it is you were doing with the 'massPos'
        % vector.
        
        g = 9.8;
        % I still have no idea why gDir can't be another axis
        gDir = 1;
        P = P + -g * translation(gDir) * massVal;  %% gDir is determined by direction of gravity in world frame 

        qdots = robot.get_joint_vars(1, false).';
        
       
        K = K + (1/2 * qdots.' * massVal * (Jvm' * Jvm) * qdots );
        % qdot is the vector of the derivatives of all joint vars
    end
end
L = simplify(K - P);
%L = simplify(K - P, 'ignoreAnalyticConstraints', true);
end

