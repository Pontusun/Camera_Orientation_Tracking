figure()
f = 250;
[nr, nc, ~, ~] = size(cam);
nr_canvas = 1000;
nc_canvas = ceil(2*pi*f)+2;
x_c_hat = nc_canvas/2;
y_c_hat = nr_canvas/2;
canvas = zeros(nr_canvas, nc_canvas, 3, 'uint8');

for i = 1:1:size(cam,4)
    img = cam(:,:,:,i);
%     if vic_flag
%         vic_i = find(t_vic > t_cam(i), 1, 'first');
%         wrb = r_vic(:,:,vic_i);
%     else
%         imu_i = find(t_imu > t_cam(i), 1, 'first');
%         wrb = rot_est(:,:,imu_i);
%     end

        imu_i = find(t_imu > t_cam(i), 1, 'first');
        wrb = rot_est(:,:,imu_i);

    [x_img, y_img] = meshgrid(1:nc, 1:nr);
    x_img = x_img(:); y_img = y_img(:); z_img = ones(size(y_img)) * f;
    P_b = bsxfun(@plus, [z_img'; -x_img'; -y_img'], [0; nc/2; nr/2]);
    P_w = wrb * P_b;
    theta = atan2(P_w(2,:), P_w(1,:));
    h       = bsxfun(@rdivide, P_w(3,:), sqrt(P_w(1,:).^2 + P_w(2,:).^2));
    x_hat   = round(-f * theta + x_c_hat);
    y_hat   = round(-f * h + y_c_hat);
    
    for k = 1:length(x_hat)
        if y_hat(k) < nr_canvas - 1 && y_hat(k) > 1
            canvas(y_hat(k), x_hat(k), :) = img(y_img(k), x_img(k), :);
        end
    end
    imshow(canvas)
    drawnow
end