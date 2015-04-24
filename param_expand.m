function [ out ] = param_expand( current_state, f )
%ODE_FUNC Simply expands the given 
   
    current_state = mat2cell(current_state,1,ones(1,numel(current_state)));
    out = f(current_state{:});
end

