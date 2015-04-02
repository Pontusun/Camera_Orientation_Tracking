clear all; close all; clc
addpath(genpath('./'))
addpath(genpath('../'))

%% load data

data_idx = 1;
vic_flag = false;
cam_flag = true;

load(sprintf('../imu/imuRaw%d.mat', data_idx));
%load('../imu/imu_test.mat');

t_imu = ts;
acc_raw = vals(1:3,:);
omg_raw = vals([5 6 4], :);
imu_raw = [acc_raw; omg_raw];

if vic_flag
    load(sprintf('../vicon/viconRot%d.mat', data_idx));
    %load(sprintf('../Test/vicon/viconRot%d.mat', data_idx));
    t_vic = ts;
    r_vic = rots;
end

if cam_flag
    load(sprintf('../cam/cam%d.mat', data_idx));
    %load('../cam/cam_test.mat');
    t_cam = ts;
end


% pre process data
[ omg_real, acc_real, bias ] = imu_data_process( imu_raw );

%  figure; plot(t_imu, acc_raw(1,:));
%  figure; plot(t_imu, acc_raw(2,:));
%  figure; plot(t_imu, acc_raw(3,:));
% figure; plot(t_imu, omg_raw(1,:));
% figure; plot(t_imu, omg_raw(2,:));
% figure; plot(t_imu, omg_raw(3,:));

%   figure; plot(t_imu, acc_real(1,:));
%   figure; plot(t_imu, acc_real(2,:));
%   figure; plot(t_imu, acc_real(3,:));
% figure; plot(t_imu, omg_real(1,:));
% figure; plot(t_imu, omg_real(2,:));
% figure; plot(t_imu, omg_real(3,:));

%% UKF

X_est = zeros(7, length(t_imu));
Z_est = zeros(6, length(t_imu));

for k = 1:length(t_imu)
    
    t = t_imu(k);
    acc = acc_real(:,k);
    omg = omg_real(:,k);
    
    if k == 1
        % Initialize 
        pt  = t; 
        X_init  = [1 0 0 0 0 0 0]';         
        X = X_init;
        X_est(:,1) = X_init;
        
        P = diag([ones(3,1)*0.0001; ones(3,1)*0.0000000001]);
        Q = diag([ones(3,1)*0.0001; ones(3,1)*0.0000000001]);
        R  = diag([ones(3,1)*0.115; ones(3,1)*0.1]);         
        fprintf('Filtering...Please Wait...\n');  
    else
        dt  = t - pt;  
        pt  = t;     
        M = [acc; omg];
        %M = acc;
        
        % Generate sigma points Xi
        S  = chol(P + Q);
        W_tmp = S * sqrt(2 * 6);
        W  = [W_tmp, -W_tmp];        
        alpha_W = vec2norm(W(1:3,:), 1);
        e_W = bsxfun(@rdivide, W(1:3,:), alpha_W);
        e_W(isnan(e_W)) = 0;
        q_W = [cos(alpha_W/2); bsxfun(@times, e_W, sin(alpha_W/2))];
        X_qi = quatmultiply(X(1:4)', q_W')';       
        Xs(1:4,:) = X_qi;
        Xs(5:7,:) = bsxfun(@plus, X(5:7,:), W(4:6,:));

        % Transform sigma points Xi to get Yi through process model
        Ys = process(Xs, omg, dt);
        
        % Calculate quaternion mean
        Y(1:4,:) = quat_mean(Ys(1:4,:));
        Y(5:7,:) = mean(Ys(5:7,:),2);
        
        % Calculate a priori state vector covariance
        [P, Wy] = sigma_covariance(Ys, Y);
        
        % Transform sigma points Yi to get Zi through measurement model
        Zs = measure(Ys);
        
        % Calculate measurement mean
        Z = sum(Zs, 2) ./ size(Zs,2);
        
        % Calculate measurement covariance
        Wz = bsxfun(@minus, Zs, Z);
        Pzz = Wz * Wz'./ size(Zs,2) /2;            

        % Calculate cross correlation matrix
        Pxz = Wy * Wz'/24 ;
        Pvv = Pzz+ R;
        
        % Calculate Kalman gain
        K = Pxz / Pvv; % Pxz_new * inv(Pvv_new);
        
        % Calculate innovation
        Inovation = M - Z;
        
        % Update state and covariance
        X = update(Y, Inovation, K);
        P  = P - K * Pvv * K';
        
        % Save state
        X_est(:,k) = X;
        Z_est(:,k) = Z;
        
    end
    
    if k == length(t_imu)
        fprintf('Filtering Complete!\n\n');
    end
end

%% Plot results

plot_result;

%% Plot rotation

plot_rotation;

%% Plot panaroma

plot_panaroma;

