function [ mean ] = quat_mean( Ys_new )
%QUAT_MEAN Summary of this function goes here
%   Detailed explanation goes here

    Mat = zeros(4,4);
    for i = 1:size(Ys_new,2)
        Mat = Mat + Ys_new(1:4,i) * Ys_new(1:4,i)';
    end
    Mat_mean = Mat ./ size(Ys_new,2);
    [V, D] = eigs(Mat_mean,1);
    if V(1) < 0
        mean = V .* (-1);
    else
        mean = V;
    end
%     mean = V;

end

