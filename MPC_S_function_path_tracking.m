function [sys,x0,str,ts] = MPC_S_function_path_tracking(t,x,u,flag)

switch flag
    case 0
         [sys, x0, str, ts] = mdlInitializeSizes;
    case 3
        sys = mdlOutputs(t,x,u);
    case {2, 4, 9}
        sys = [];
    % Unexpected flags
    otherwise
        error(['UNhandled flag = ', num2str(flag)]);
         
end

function [sys, x0, str, ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates = 0;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 4;
    sizes.NumInputs = 10;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);    
    x0 = [];
    str = [];
    ts = [0.1, 0];

    % 清除图形并初始化绘图
    initializePlot();

function sys = mdlOutputs(t,x,u) 
% 获取车辆当前位置状态 x_car = [X; Y; phi]

X_position = u(1);
Y_position = u(2);
theta_position = u(3);
Ux = u(4);
beta = u(5);
r = u(6);
V = Ux / (cos(beta) + eps);
Uy = V * sin(beta);
Time = u(7);
%上一时刻的车辆
u_prev_init = [u(8); u(9); u(10)];
% 初始化
opti = casadi.Opti();
% 预测步数
N = 20;
Ts = 0.1;
% 初始参考轨迹 Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
Ref_path0 = opti.parameter(6, N);
X0 = opti.parameter(3, 1);
predict_data = zeros(3, N);
% 状态变量 X = [X - X_r; Y - Y_r; phi - phi_r]
% 控制变量 U = [vx - vx_r; vy - vy_r; r - r_r]
% 参考轨迹匹配 Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
X = opti.variable(3, N + 1);
U = opti.variable(3, N);
U_prev = opti.parameter(3, 1); % 前一时刻的控制信号
delta_U = opti.variable(3, N);
obstacle_position = [-5; 10000]; % 障碍物位置

%% MPC部分
% 状态变量 X = [X - X_r; Y - Y_r; phi - phi_r]
x_car = [0; 0; 0];
x_car(1) = X_position;
x_car(2) = Y_position;
x_car(3) = theta_position;
J=0;
% 初始化参考轨迹
Ref_path_Init = zeros(6, N);
distances = zeros(1, N); % 用于存储每个点的距离

for k = 1:N
    % 计算当前位置
    delta_t = (k-1) * Ts;
    predicted_x = x_car(1) + V * delta_t * cos(x_car(3) + r * delta_t);
    predicted_y = x_car(2) + V * delta_t * sin(x_car(3) + r * delta_t);
    currentPosition = [predicted_x, predicted_y];
    nearestPoint = findNearestPoint(currentPosition, x_car(3), Time);
    
    % 获取最近点的参考轨迹
    X_r  = nearestPoint(1);
    Y_r  = nearestPoint(2);
    theta_r = nearestPoint(3);  
    Ux_r = nearestPoint(4);
    Uy_r = nearestPoint(5);
    r_r  = nearestPoint(6);  
    % 计算当前距离参考轨迹的距离
    distance = sqrt((predicted_x - X_r)^2 + (predicted_y - Y_r)^2);
    theta_error = x_car(3) - theta_r;
    distances(k) = distance;
    
    % 动态调整 Q_M 和 R
    % if distance > 2 || abs(theta_error) > 0.2  % 设定一个阈值
    if Time < 5  % 设定一个阈值
        Q_M = 1e01 * ([5 0 0;
                        0 5 0;
                        0 0 0.5]);
        R   = 1e010 * ([1 0 0;
                        0 1000 0;
                        0 0 1]);

        disp('Time:')
        disp(Time)
        disp('distance > threshold')
        opti.subject_to( -2 <= U(3, :)+Ref_path0(6,:) <= 2);
    else
        Q_M = 1e01 * ([5 0 0;
                        0 5 0;
                        0 0 1]);
        R = 1e01 * ([1 0 0;
                        0 5 0;
                        0 0 1]);
        disp('Time:')
        disp(Time)
        disp('distance < threshold')
        opti.subject_to( -2 <= U(3, :)+Ref_path0(6,:) <= 2);
    end
    % 填充参考轨迹矩阵
    Ref_path_Init(1, k) = X_r;
    Ref_path_Init(2, k) = Y_r;
    Ref_path_Init(3, k) = theta_r;  
    Ref_path_Init(4, k) = Ux_r;
    Ref_path_Init(5, k) = Uy_r;
    Ref_path_Init(6, k) = r_r;  
    
    predict_data(1, k) = X_r;
    predict_data(2, k) = Y_r;
    predict_data(3, k) = theta_r;

    
    % 模型预测控制部分
    if k == 1
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), U_prev, Ref_path0(:, k), Ts));
    else
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), U(:, k-1), Ref_path0(:, k), Ts));
    end
    
    % 计算目标函数
    J = J + X(:, k)' * Q_M * X(:, k) + U(:, 1)' * R * U(:, 1);
    
    % 安全约束
    epsilon = 1e-6; % 小的正数，防止除零
    penalty = 100000 / ( h(obstacle_position, X(:, k), U(:, k), Ref_path0(:, k) )^2 + epsilon ); % epsilon 是一个小的正数，防止除零
    J = J + penalty;
end
    % % 计算 delta_U
    % opti.subject_to(delta_U(1, 1) == (U(1, 1)+Ref_path0(4, 1) - Ux) / Ts);
    % opti.subject_to(delta_U(2, 1) == (U(2, 1)+Ref_path0(5, 1) - Uy) / Ts);
    % opti.subject_to(delta_U(3, 1) == (U(3, 1)+Ref_path0(6, 1) - r) / Ts);
    % for k = 1:N-1
    %     opti.subject_to(delta_U(:, k) == (U(:, k+1) - U(:, k)) / Ts);
    % end
    % %% 0
    % max_delta_U = [10000; 50000; 10000]; % 控制增量的最大值
    % % max_delta_U = [10; 1; 10]; % 控制增量的最大值
    % % 施加控制增量约束
    % opti.subject_to(-max_delta_U(1) <= delta_U(1, :) <= max_delta_U(1));
    % opti.subject_to(-max_delta_U(2) <= delta_U(2, :) <= max_delta_U(2));
    % opti.subject_to(-max_delta_U(3) <= delta_U(3, :) <= max_delta_U(3));
%%
% 结束约束条件
opti.subject_to(X(:, 1) == X0);

opti.subject_to(  0 <= U(1, :)+Ref_path0(4,:) <= 14);
opti.subject_to( -8 <= U(2, :)+Ref_path0(5,:) <= 8);



% 优化问题设置
opti.minimize(J);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0; % 0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
opti.solver('ipopt', opts);


% 初始化状态变量 x = [X - X_r; Y - Y_r; phi - phi_r]
x_Init = x_car(1:3, 1) - Ref_path_Init(1:3, 1);
% 控制变量 U = [vx - vx_r; vy - vy_r; r - r_r]
% 设置优化变量的初始值
opti.set_value(X0, x_Init);
opti.set_value(Ref_path0, Ref_path_Init); 
opti.set_value(U_prev, u_prev_init); % 设置前一时刻的控制信号初始值
sol = opti.solve();
uout = sol.value(U);
X_out = sol.value(X);
% 绘制状态量和变化率


predict_data(1, :) = predict_data(1, :) + X_out(1, 1:N);
predict_data(2, :) = predict_data(2, :) + X_out(2, 1:N);
predict_data(3, :) = predict_data(3, :) + X_out(3, 1:N);
u_car = uout + Ref_path_Init(4:6, :);
%打印
% disp(u_car)

% 计算实际控制量 u_car = [vx; vy; r]
% 应用控制量

plot(x_car(1), x_car(2), '*');
hold on;
% plot(X_out(1, 1:N) + Ref_path_Init(1, :), X_out(2, 1:N) + Ref_path_Init(2, :), 'r');
plot(predict_data(1, :), predict_data(2, :), 'r');

% 打印调试信息
sys(1) = u_car(1,1);  % 同时 sys 保存当前的控制值，sys[1] 是系统输出，用于 Simulink 的 S-function 接口
sys(2) = atan(u_car(2,1) / (u_car(1,1) + eps));
sys(3) = u_car(3,1);
sys(4) = 6000;

function h_val = h(obstacle_position,X, U, Refpath0)
    % 定义控制障碍函数 CBF
    % 假设有一个障碍物，确保车辆与障碍物之间的距离大于某个安全值
    
    % 计算车辆当前位置与障碍物之间的距离
    distance = norm(X(1:2,:) + Refpath0(1:2,:) - obstacle_position);
    % 计算 CBF 值
    h_val = distance ;

function initializePlot()
    clf;
    figure(1);
    [~, trajectory] = findNearestPoint([0, 0], 0, 0);
    plot(trajectory(:, 1), trajectory(:, 2), 'b');
    hold on;
    grid on;
    % 画圆心在 (0, 1) 半径为 1 的圆
    theta = linspace(0, 2*pi, 100);
    x_circle = 2 * cos(theta);
    y_circle = 2 * sin(theta) + 1;
    plot(x_circle, y_circle, 'r');

    % 画圆心在 (0, 1) 半径为 1 的圆
    theta = linspace(0, 2*pi, 100);
    x_circle = 27 * cos(theta) -48;
    y_circle = 27 * sin(theta) -46;
    plot(x_circle, y_circle, 'b');
    


