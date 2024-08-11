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
sizes.NumContStates = 0;    % 连续状态的数量
sizes.NumDiscStates = 0;    % 离散状态的数量
sizes.NumOutputs = 4;       % 输出端口的数量
sizes.NumInputs = 10;       % 输入端口的数量
sizes.DirFeedthrough = 1;   % 直接馈通标志，1表示存在直接馈通
sizes.NumSampleTimes = 1;   % 采样时间的数量

sys = simsizes(sizes);    
x0 = [];
str = [];
ts = [0.1,0];
% ts = [ ];
% 清除图形
clf;
figure(1);
[~,trajectory]=findNearestPoint([0,0], 0 ,0);
plot(trajectory(:,1),trajectory(:,2),'b');

hold on;
grid on;
% 画圆心在 (-0.5, -25) 半径为 1 的圆
theta = linspace(0, 2*pi, 100);
x_circle = 0 +1* cos(theta);
y_circle = 0 + 1*sin(theta);
plot(x_circle, y_circle, 'r');

function sys = mdlOutputs(t,x,u) 
    %%

Time = u(7);
% 初始化
opti = casadi.Opti();
% 预测步数
N = 20;
Ts = 0.1;
% 初始参考轨迹 Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
Ref_path0 = opti.parameter(6, N);
X0 = opti.parameter(3, 1);
predict_data = zeros(2, N);
% 状态变量 X = [X - X_r; Y - Y_r; phi - phi_r]
% 控制变量 U = [vx - vx_r; vy - vy_r; r - r_r]
% 参考轨迹匹配 Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
X = opti.variable(3, N + 1);
U = opti.variable(3, N);
delta_U = opti.variable(3, N);
% 控制增量变量
J = 0;
if Time < 0.5
    Q_M = 1e01 * ([5 0 0;
                   0 5 0;
                   0 0 1]);
    % 状态权重矩阵
    R = 1e04 * ([1 0 0;
                 0 5 0;
                 0 0 1]);
    % 模型预测控制部分
    for k = 1:N    
        % 预测轨迹，确保每一步的状态满足模型
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), Ref_path0(:, k), Ts)); 
        J = J + X(:, k)' * Q_M * X(:, k) + U(:, 1)' * R * U(:, 1);      
        opti.subject_to(delta_U(:, k) == 0);
    end

else
    Q_M = 1e03 * ([5 0 0;
                   0 5 0;
                   0 0 1]);
    % 状态权重矩阵
    R = 1e01 * ([1 0 0;
                 0 5000 0;
                 0 0 1]);
    % 模型预测控制部分
    for k = 1:N    
        % 预测轨迹，确保每一步的状态满足模型
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), Ref_path0(:, k), Ts));  
        % 安全约束
        % opti.subject_to(h(X(:, k), U(:, k), Ref_path0(:, k)) >= 0);
        % J = J + X(:, k)' * Q_M * X(:, k) + U(:, 1)' * R * U(:, 1);    
        epsilon = 1e-6; % 小的正数，防止除零
        % 原始目标函数
        J = J + X(:, k)' * Q_M * X(:, k) + U(:, 1)' * R * U(:, 1);
        % 当 h 接近 0 时的惩罚
        penalty = 100000 / (h(X(:, k), U(:, k), Ref_path0(:, k))^2 + epsilon); % epsilon 是一个小的正数，防止除零
        J = J + penalty;    
    end

    % 计算 delta_U
    opti.subject_to(delta_U(1, 1) == (U(1, 1) - u(8)) / Ts);
    opti.subject_to(delta_U(2, 1) == (U(2, 1) - u(9)) / Ts);
    opti.subject_to(delta_U(3, 1) == (U(3, 1) - u(10)) / Ts);
    for k = 1:N-1
        opti.subject_to(delta_U(:, k) == (U(:, k+1) - U(:, k)) / Ts);
    end
    %% 

    max_delta_U = [200; 200; 200]; % 控制增量的最大值
    % 施加控制增量约束
    opti.subject_to(-max_delta_U(1) <= delta_U(1, :) <= max_delta_U(1));
    opti.subject_to(-max_delta_U(2) <= delta_U(2, :) <= max_delta_U(2));
    opti.subject_to(-max_delta_U(3) <= delta_U(3, :) <= max_delta_U(3));
end
%%
% 结束约束条件
opti.subject_to(X(:, 1) == X0);

opti.subject_to(  0 <= U(1, :)+Ref_path0(4,:) <= 14);
opti.subject_to( -6 <= U(2, :)+Ref_path0(5,:) <= 6);
opti.subject_to( -1.25 <= U(3, :)+Ref_path0(6,:) <= 1.25);


% 优化问题设置
opti.minimize(J);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0; % 0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
opti.solver('ipopt', opts);

%% MPC部分
% 状态变量 X = [X - X_r; Y - Y_r; phi - phi_r]
x_car = [0; 0; 0];
% 获取车辆当前位置状态 x_car = [X; Y; phi]
x_car(1) = u(1);
x_car(2) = u(2);
x_car(3) = u(3);
Ux = u(4);
beta = u(5);
r = u(6);
V = Ux / (cos(beta) + eps);

% 初始化参考轨迹
Ref_path_Init = zeros(6, N);
for i = 1:N
    % 计算当前位置
    currentPosition = [x_car(1) + (i-1) * Ts * V * cos(x_car(3) + beta), x_car(2) + (i-1) * Ts * V * sin(x_car(3) + beta)];

    nearestPoint = findNearestPoint(currentPosition, x_car(3), Time);
    % 获取最近点的参考轨迹
    X_r = nearestPoint(1);
    Y_r = nearestPoint(2);
    theta_r = nearestPoint(3);  
    Ux_r = nearestPoint(4);
    Uy_r = nearestPoint(5);
    r_r = nearestPoint(6);  
    
    % 填充参考轨迹矩阵
    Ref_path_Init(1, i) = X_r;
    Ref_path_Init(2, i) = Y_r;
    Ref_path_Init(3, i) = theta_r;  
    Ref_path_Init(4, i) = Ux_r;
    Ref_path_Init(5, i) = Uy_r;
    Ref_path_Init(6, i) = r_r;  
    
    predict_data(1, i) = X_r;
    predict_data(2, i) = Y_r;
end

% 初始化状态变量 x = [X - X_r; Y - Y_r; phi - phi_r]
x_Init = x_car(1:3, 1) - Ref_path_Init(1:3, 1);
% 控制变量 U = [vx - vx_r; vy - vy_r; r - r_r]

% 设置优化变量的初始值
opti.set_value(X0, x_Init);
opti.set_value(Ref_path0, Ref_path_Init); 

sol = opti.solve();
uout = sol.value(U);
X_out = sol.value(X);

predict_data(1, :) = predict_data(1, :) + X_out(1, 1:N);
predict_data(2, :) = predict_data(2, :) + X_out(2, 1:N);
% 计算实际控制量 u_car = [vx; vy; r]
% 应用控制量
u_car = uout(:, 1) + Ref_path_Init(4:6, 1);
%打印
disp("Time:");
disp(Time);
disp("u_car:");
disp(uout + Ref_path_Init(4:6,:));
plot(x_car(1), x_car(2), '*');
hold on;
% plot(X_out(1, 1:N) + Ref_path_Init(1, :), X_out(2, 1:N) + Ref_path_Init(2, :), 'r');
plot(predict_data(1, :), predict_data(2, :), 'r');

% axis([-25 25 -25 25]);
% 打印调试信息

sys(1) = u_car(1);  % 同时 sys 保存当前的控制值，sys[1] 是系统输出，用于 Simulink 的 S-function 接口
sys(2) = atan(u_car(2) / (u_car(1) + eps));
sys(3) = u_car(3);
sys(4) = 6000;

function h_val = h(X, U, Refpath0)
    % 定义控制障碍函数 CBF
    % 假设有一个障碍物，确保车辆与障碍物之间的距离大于某个安全值
    obstacle_position = [-2; 2]; % 障碍物位置
    safe_distance = 1; % 安全距离
    
    % 计算车辆当前位置与障碍物之间的距离
    distance = norm(X(1:2) + Refpath0(1:2) - obstacle_position);
    
    % 计算 CBF 值
    h_val = distance - safe_distance;



