function [ X ] = update( X, V, K )
%UPDATE Summary of this function goes here
%   Detailed explanation goes here

U = K * V;
U_r = U(1:3); 
U_b = U(4:6);

X_q = X(1:4);
alpha_U = norm(U_r,2);
e_U = U_r / norm(U_r,2);
q_U = [cos(alpha_U/2); e_U * sin(alpha_U/2)];
X_q = quatmultiply(X_q', q_U')';

X_b = X(5:7);
X_b = X_b + U_b;

X = [X_q; X_b];

end

