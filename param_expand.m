function [ out ] = param_expand( c, f )
%ODE_FUNC Simply expands the given 
   
    switch numel(c)
        case 1
            out = f(c);
        case 2
            out = f(c(1), c(2));
        case 4
            out = f(c(1), c(2), ...
                    c(3), c(4));
        case 6
            out = f(c(1), c(2), ...
                    c(3), c(4), ...
                    c(5), c(6));
        otherwise
            disp(strcat('Param Expand for', ' ', int2str(numel(c)), ' elements not implemented!')) 
    end
    %current_state = mat2cell(current_state,1,ones(1,numel(current_state)));
    %out = f(current_state{:});
end

