function [ q ] = get_joint_var( obj, link_no, order, is_time )
    
if ~is_time
    switch order
        case 0
            if ~isempty(obj.dh_params(link_no).q)
                q = obj.dh_params(link_no).q;
            end
        case 1
            if ~isempty(obj.dh_params(link_no).qd)
                q = obj.dh_params(link_no).qd;
            end
        case 2
            if ~isempty(obj.dh_params(link_no).qdd)
                q = obj.dh_params(link_no).qdd;
            end
    end
else
    switch order
        case 0
            if ~isempty(obj.dh_params(link_no).q_t)
                q = obj.dh_params(link_no).q_t;
            end
        case 1
            if ~isempty(obj.dh_params(link_no).qd_t)
                q = obj.dh_params(link_no).qd_t;
            end
        case 2
            if ~isempty(obj.dh_params(link_no).qdd_t)
                q = obj.dh_params(link_no).qdd_t;
            end
    end
end

