function [ omg, acc, bias ] = imu_data_process( imu_raw )
%PRE_PROCESS Summary of this function goes here
%   Detailed explanation goes here

Vref = 3300;
acc_sensitivity = 300; % 300mv/g
%omg_sensitivity = 3.3333; % 3.3333 degree/s
omg_sensitivity = 3.3333 * 180 / pi;

acc_scale = Vref / 1023 / acc_sensitivity * [-1; -1; 1]; % = 0.0108 (x, y axis fliped)
omg_scale = Vref / 1023 / omg_sensitivity; % = 0.0169

bias = mean(imu_raw(:, 1 : 200), 2);
bias(3) = bias(3) - 1/ acc_scale(3) ;% fix the gravity

acc = bsxfun(@times, bsxfun(@minus, imu_raw(1:3,:), bias(1:3)), acc_scale);
omg = bsxfun(@minus, imu_raw(4:6,:), bias(4:6)) * omg_scale;

end

