function [ robot ] = add_mass( robot, link_no, mass, pos )
%ADD_MASS Summary of this function goes here
%   Detailed explanation goes here
    
robot.dh_params(link_no).masses = [robot.dh_params(link_no).masses, {mass, pos}];

end

