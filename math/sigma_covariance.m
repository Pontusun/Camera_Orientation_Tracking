function [ cov, W ] = sigma_covariance( X_sigma, X_mean )
%QUAD_COV Summary of this function goes here
%   Detailed explanation goes here
q_res = quatmultiply(X_sigma(1:4,:)', quatconj(X_mean(1:4,:)'))';
omg_res = bsxfun(@minus, X_sigma(5:7,:), X_mean(5:7,1));

cov = [q_res(2:4,:); omg_res] * [q_res(2:4,:); omg_res]'./ size(X_sigma, 2)/2;

W = [ q_res(2:4,:); omg_res ];

end

