function x_update = vehicle_path_tracking_model(x, u, u_prev, Ref_path, Ts, tau)
    % 提取参考轨迹
    X_r = Ref_path(1);
    Y_r = Ref_path(2);
    phi_r = Ref_path(3);
    vx_r = Ref_path(4);
    vy_r = Ref_path(5);
    r_r = Ref_path(6);

    % 轨迹跟踪模型
    A = [0 0 -vx_r*sin(phi_r)-vy_r*cos(phi_r);
         0 0  vx_r*cos(phi_r)-vy_r*sin(phi_r);
         0 0               0            ];   
    B = [cos(phi_r) -sin(phi_r) 0;
         sin(phi_r)  cos(phi_r) 0;
         0             0       1];   
    C = eye(3); 

    % 控制信号延迟模型 (一阶滞后)
    u_delayed = (tau) * u + (1 - tau) * u_prev;

    % 计算状态导数
    x_dot = A * x + B * u_delayed;

    % 更新状态
    x_update = x + x_dot * Ts;
end