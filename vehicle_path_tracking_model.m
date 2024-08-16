function x_update = vehicle_path_tracking_model(x, u, u_prev, Ref_path, Ts)
     % 提取参考轨迹
     X_r = Ref_path(1);
     Y_r = Ref_path(2);
     phi_r = Ref_path(3);
     vx_r = Ref_path(4);
     vy_r = Ref_path(5);
     r_r = Ref_path(6);

     phi = x(3);
     vx = u_prev(1);
     vy = u_prev(2);
     r = u_prev(3);
     % 轨迹跟踪模型
     A = [0 0 -vx_r*sin(phi_r)-vy_r*cos(phi_r);
          0 0  vx_r*cos(phi_r)-vy_r*sin(phi_r);
          0 0               0            ];   
     B = [cos(phi_r) -sin(phi_r) 0;
          sin(phi_r)  cos(phi_r) 0;
          0             0       1];   
     C = eye(3); 
     % 控制信号延迟模型 (一阶滞后)
     % 计算状态导数

     x_dot = A * x + B * u_prev;
     % x_dot_N = zeros(3, 1);
     % x_dot_N(1) = vx * phi - vy * phi;
     % x_dot_N(2) = vx * phi + vy * phi;
     % x_dot_N(3) = r;
     % disp('error:')
     % disp(x_dot - x_dot_N)
     %alpha越大，控制信号越接近当前时刻的控制信号
     alpha = 0.3; 
     % 更新状态
     x_update = (1 - alpha) * x + alpha * (x + x_dot * Ts);
end