classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dh_params
        function_cache
        sym_cache
    end
    
    methods
        function [ obj ] = add_link(obj, dh, mass, joint_var, is_prismatic)
            if nargin < 5
                is_prismatic = false;
            end
            T = DHToMatrix_vec(dh);
            s = struct('DH', dh, 'mass', mass,... 
                'T', T, 'q', joint_var, 'prismatic', is_prismatic);
            obj.dh_params = [obj.dh_params, s];
        end
        
        function [n] = num_links(obj)
            n = size(obj.dh_params, 2);
        end
        
        function [T] = get_link_tf(obj, link_no)
            T = obj.dh_params(link_no).T;
        end
        
        function [q_vec] = get_joint_vars(obj)
             q_vec = [obj.dh_params(:).q];
        end
        
        function [q_var] = get_joint_var(obj, link_no)
             q_var = obj.dh_params(link_no).q;
        end
    end
end

