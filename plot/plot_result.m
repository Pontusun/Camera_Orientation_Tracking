
rot_est = quat2dcm(quatconj(X_est(1:4,:)'));
eul_est = rots2rpy(rot_est);
eul_est = fix_eul(eul_est);

if vic_flag
    eul_vic = rots2rpy(r_vic);
    eul_vic = fix_eul(eul_vic);
    h_eul = figure();
    plot_state(h_eul, t_vic - min(t_imu(1), t_vic(1)), eul_vic, 'eul', 'vic');
    plot_state(h_eul, t_imu - min(t_imu(1), t_vic(1)), eul_est, 'eul', 'est');
    h_mea = figure();
    plot_state(h_mea, t_imu - min(t_imu(1), t_vic(1)), acc_real, 'acc', 'mea');
    plot_state(h_mea, t_imu - min(t_imu(1), t_vic(1)), Z_est(1:3,:), 'acc', 'est');
else
    h_eul = figure();
    plot_state(h_eul, t_imu - t_imu(1), eul_est, 'eul', 'est');
    h_mea = figure();
    plot_state(h_mea, t_imu - t_imu(1), Z_est(1:3,:), 'acc', 'est');
end