function [p, R] = TF_Pos_Rot(T)
p = T(1:3, 4);
R = T(1:3, 1:3);
end