classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dh_params
        function_cache
        sym_cache
    end
    
    methods
        function [n] = num_links(obj)
            n = size(obj.dh_params, 2);
        end
        
        function [n] = num_joints(obj)
            n = numel(obj.get_joint_vars(0, false));
        end
        
        function [n] = num_masses(obj, link_no)
            n = size(obj.dh_params(link_no).masses, 1);
        end
        
        function [n] = get_masses(obj, link_no)
            n = obj.dh_params(link_no).masses;
        end
        
        function [T] = get_link_tf(obj, link_no)
            T = obj.dh_params(link_no).T;
        end
        
        function [T] = has_joint_var(obj, link_no, order, is_time)
            T = ~isempty(obj.get_joint_var(link_no, order, is_time));
        end
    end
end

