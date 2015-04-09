function [ robot ] = add_link( robot, dh, mass, varargin )
    p = inputParser;

    addRequired(p,'robot');
    addRequired(p,'dh');
    addRequired(p,'mass');
    
    addParameter(p,'joint_var', [], @(a) ~isnumeric(a));
    addParameter(p,'prismatic', false, @islogical);

    parse(p, robot, dh, mass, varargin{:});

    T = DHToMatrix_vec(dh);
    s = struct('DH', dh, 'mass', mass,... 
        'T', T, 'q', p.Results.joint_var );

    robot.dh_params = [robot.dh_params, s];
        
end

