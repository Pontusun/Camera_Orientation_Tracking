function [ Zs ] = measure( Ys )
%MEASURE Summary of this function goes here
%   Detailed explanation goes here
Zs = zeros(6, size(Ys,2));
n_sigma = size(Ys,2);
for i = 1:n_sigma
    q   = Ys(1:4,i);
    g   = [0; 0; 0; 1];
    g_q = quatmultiply(quatmultiply(quatconj(q'), g'), q');
    Zs(1:3,i)  = g_q(2:4)';
    Zs(4:6,i)  = Ys(5:7,i);
end

end

