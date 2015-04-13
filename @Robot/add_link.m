function [ robot ] = add_link( robot, dh, varargin )
    p = inputParser;

    addRequired(p,'robot');
    addRequired(p,'dh');
    
    addParameter(p,'joint_var', [], @(a) ~isnumeric(a));
    addParameter(p,'joint_var_dot', [], @(a) ~isnumeric(a));
    addParameter(p,'prismatic', false, @islogical);

    parse(p, robot, dh, varargin{:});

    T = DHToMatrix_vec(dh);
    s = struct('DH', dh, 'masses', [], 'T', T, ...
        'q', p.Results.joint_var, 'qdot', p.Results.joint_var_dot);

    robot.dh_params = [robot.dh_params, s];
        
end

