function [ Ys ] = process( Xs, U, dt )
%PROCESS Summary of this function goes here
%   Detailed explanation goes here

n_sigma = size(Xs,2);
Ys = zeros(size(Xs));

for i = 1:n_sigma
    
    q_k = Xs(1:4,i);
    b_k = Xs(5:7,i);
    w_k = U + b_k;

    alpha_d = norm(w_k,2)*dt;
    e_d = w_k / norm(w_k,2);
    % e_d(isnan(e_d)) = 0;
    q_d = [cos(alpha_d/2); e_d*sin(alpha_d/2)];

    q_k1 = quatmultiply(q_k', q_d')';
    b_k1 = b_k;

    Ys(:,i) = [q_k1; b_k1];   
end


end

