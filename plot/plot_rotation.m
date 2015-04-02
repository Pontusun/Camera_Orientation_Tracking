
figure();
    % Find start and stop indices
    if vic_flag 
        if t_imu(1) > t_vic(1)
            imu_start_i = 1;
        else
            imu_start_i = find(t_imu > t_vic(1), 1 ,'first');
        end
        if t_imu(end) < t_vic(end)
            imu_stop_i = length(t_imu);
        else
            imu_stop_i = find(t_imu < t_vic(end), 1, 'last');
        end
        
    else
        imu_start_i = 1;
        imu_stop_i = length(t_imu);
    end
    % Start animation
    for i = imu_start_i : imu_stop_i
        if vic_flag
            vic_i = find(t_imu(i) < t_vic, 1, 'first');
            if isempty(vic_i), vic_i = length(t_vic); end
        end
        if i == imu_start_i
            if vic_flag
                subplot(1,2,1)
                h_vic = myrotplot(r_vic(:,:,vic_i));
                title('Vicon')
                subplot(1,2,2)
                h_est = myrotplot(rot_est(:,:,i));
                title('Estimation')
            else
                h_est = myrotplot(rot_est(:,:,i));
                title('Estimation')
            end
        else
            if vic_flag
                myrotplot(r_vic(:,:,vic_i), h_vic);
                myrotplot(rot_est(:,:,i), h_est);
            else
                myrotplot(rot_est(:,:,i), h_est);
            end
        end
        drawnow
    end