function [ robot ] = add_link( robot, dh, varargin )
    p = inputParser;

    addRequired(p,'robot');
    addRequired(p,'dh');
    addParameter(p,'joint_var', [], @(a) ~isnumeric(a));
    addParameter(p,'prismatic', false, @islogical);

    parse(p, robot, dh, varargin{:});

    T = DHToMatrix_vec(dh);
    q = p.Results.joint_var;
    q_t = sym(strcat([char(q), '(t)']));
    qd = sym(strcat([char(q), 'd']), 'real');
    qd_t = sym(strcat([char(q), 'd(t)']));
    qdd = sym(strcat([char(q), 'dd']), 'real');
    qdd_t = sym(strcat([char(q), 'dd(t)']));
    qint = sym(strcat([char(q), 'int']), 'real');
    qint_t = sym(strcat([char(q), 'qint(t)']));
    tau = sym(strcat([ 'tau', num2str(size(robot.dh_params, 2)) ]), 'real');
    s = struct('DH', dh, ...
        'masses', [], ...
        'T', T, ...
        'q', q, ...
        'q_t', q_t, ...
        'qd', qd, ...
        'qd_t', qd_t, ...
        'qdd', qdd, ...
        'qdd_t', qdd_t, ...
        'qint', qint, ...
        'qint_t', qint_t, ...
        'tau', tau);

    robot.dh_params = [robot.dh_params, s];
        
end

