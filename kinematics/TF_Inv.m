function [ inv ] = TF_Inv( tf )
%TFinv computes the inverse of a Homogenous transformation matrix
r = tf(1:3,1:3);
t = tf(1:3,4);
inv = [r', -r'*t; 0, 0, 0, 1];
end

