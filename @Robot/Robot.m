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
            n = numel(obj.get_joint_vars());
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
        
        function [q_vec] = get_joint_vars(obj)
            q_vec = [];
            for q_idx = 1:obj.num_links()
                if ~isempty(obj.dh_params(q_idx).q)
                    q_vec = [q_vec, obj.dh_params(q_idx).q];
                end
            end
        end
        
        function [ has_var ] = has_joint_var(obj, link_no)
            has_var = numel(obj.get_joint_var(link_no)) ~= 0;
        end
        
        function [ q_var ] = get_joint_var(obj, link_no)
            q_var = obj.dh_params(link_no).q;
        end
        
        function [ q_var ] = get_joint_var_dot(obj, link_no)
            q_var = obj.dh_params(link_no).qdot;
        end
    end
end

