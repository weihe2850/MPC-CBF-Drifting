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

    % ���ͼ�β���ʼ����ͼ
    initializePlot();
    figure(1);
function sys = mdlOutputs(t,x,u) 
% ��ȡ������ǰλ��״̬ x_car = [X; Y; phi]

X_position = u(1);
Y_position = u(2);
theta_position = u(3);
Ux = u(4);
beta = u(5);
r = u(6);
V = Ux / (cos(beta) + eps);
Uy = V * sin(beta);
Time = u(7);
%��һʱ�̵ĳ���
u_prev_init = [u(8); u(9); u(10)];
% ��ʼ��
opti = casadi.Opti();
% Ԥ�ⲽ��
N = 20;
Ts = 0.1;
% ��ʼ�ο��켣 Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
Ref_path0 = opti.parameter(6, N);
X0 = opti.parameter(3, 1);
predict_data = zeros(3, N);
% ״̬���� X = [X - X_r; Y - Y_r; phi - phi_r]
% ���Ʊ��� U = [vx - vx_r; vy - vy_r; r - r_r]
% �ο��켣ƥ�� Ref_path = [X_r; Y_r; phi_r; vx_r; vy_r; r_r]
X = opti.variable(3, N + 1);
U = opti.variable(3, N);
U_prev = opti.parameter(3, 1); % ǰһʱ�̵Ŀ����ź�
delta_U = opti.variable(3, N);
obstacle_position = [-5; -38]; % �ϰ���λ��

%% MPC����
% ״̬���� X = [X - X_r; Y - Y_r; phi - phi_r]
x_car = [0; 0; 0];
x_car(1) = X_position;
x_car(2) = Y_position;
x_car(3) = theta_position;
J=0;
% ��ʼ���ο��켣
Ref_path_Init = zeros(6, N);
distances = zeros(1, N); % ���ڴ洢ÿ����ľ���

for k = 1:N
    % ���㵱ǰλ��
    delta_t = (k-1) * Ts;
    predicted_x = x_car(1) + V * delta_t * cos(x_car(3) + r * delta_t);
    predicted_y = x_car(2) + V * delta_t * sin(x_car(3) + r * delta_t);
    currentPosition = [predicted_x, predicted_y];
    nearestPoint = findNearestPoint(currentPosition, x_car(3), Time);
    
    % ��ȡ�����Ĳο��켣
    X_r  = nearestPoint(1);
    Y_r  = nearestPoint(2);
    theta_r = nearestPoint(3);  
    Ux_r = nearestPoint(4);
    Uy_r = nearestPoint(5);
    r_r  = nearestPoint(6);  
    % ���㵱ǰ����ο��켣�ľ���
    distance = sqrt((predicted_x - X_r)^2 + (predicted_y - Y_r)^2);
    theta_error = x_car(3) - theta_r;
    distances(k) = distance;

    % ��̬���� Q_M �� R
    % if distance > 2 || abs(theta_error) > 0.2  % �趨һ����ֵ
    if Time <4  % �趨һ����ֵ
        Q_M = 1e01 * ([5 0 0;
                        0 5 0;
                        0 0 0.5]);
        R   = 1e010 * ([1 0 0;
                        0 1000 0;
                        0 0 1]);
    else
        Q_M = 1e01 * ([5 0 0;
                        0 5 0;
                        0 0 1]);
        R = 1e01 * ([1 0 0;
                        0 5 0;
                        0 0 1]);
    end

    % ���ο��켣����
    Ref_path_Init(1, k) = X_r;
    Ref_path_Init(2, k) = Y_r;
    Ref_path_Init(3, k) = theta_r;  
    Ref_path_Init(4, k) = Ux_r;
    Ref_path_Init(5, k) = Uy_r;
    Ref_path_Init(6, k) = r_r;  
    
    predict_data(1, k) = X_r;
    predict_data(2, k) = Y_r;
    predict_data(3, k) = theta_r;

    
    % ģ��Ԥ����Ʋ���
    if k == 1
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), U_prev, Ref_path0(:, k), Ts));
    else
        opti.subject_to(X(:, k+1) == vehicle_path_tracking_model(X(:, k), U(:, k), U(:, k-1), Ref_path0(:, k), Ts));
    end
    
    % �������
    r_car = U(3, k)  + r_r;
    Uy_car = U(2, k)  + Uy_r;
    Ux_car = U(1, k)  + Ux_r;
    % ʹ�� if_else ���������жϺ�Լ��
    % drift_condition = abs(U2) > 0.2;
    % opti.subject_to(drift_condition * abs(U3) >= drift_condition * 0.5); % Ư��״̬�º�����ٶȵ�Լ��
    % opti.subject_to(drift_condition * abs(U3 * U2) <= drift_condition * 0); % Ư��״̬�º�����ٶȵ�Լ��

    % non_drift_condition = abs(U3_car) < 0.3;
    % opti.subject_to(non_drift_condition * abs(U2_car) == non_drift_condition * 0); % ��Ư��״̬�²໬�ǵ�Լ��

    % ����Ư��״̬�ͷ�Ư��״̬����Լ���ͷ�
    drift_control_penalty = calculate_penalties(Ux_car, Uy_car, r_car);

    % ����Ŀ�꺯��
    J = J + X(:, k)' * Q_M * X(:, k) + U(:, 1)' * R * U(:, 1) + drift_control_penalty;
    
    % % % % ��ȫԼ��
    if 15 < Time   && Time < 30
    epsilon = 1e-6; % С����������ֹ����
    penalty = 50000 / ( h(obstacle_position, X(:, k), U(:, k), Ref_path0(:, k) )^2 + epsilon ); % epsilon ��һ��С����������ֹ����
    J = J + penalty;

    end
end
initializePlot()
    % % ���� delta_U
    % opti.subject_to(delta_U(1, 1) == (U(1, 1)+Ref_path0(4, 1) - Ux) / Ts);
    % opti.subject_to(delta_U(2, 1) == (U(2, 1)+Ref_path0(5, 1) - Uy) / Ts);
    % opti.subject_to(delta_U(3, 1) == (U(3, 1)+Ref_path0(6, 1) - r) / Ts);
    % for k = 1:N-1
    %     opti.subject_to(delta_U(:, k) == (U(:, k+1) - U(:, k)) / Ts);
    % end
    % %% 0
    % max_delta_U = [10000; 50000; 10000]; % �������������ֵ
    % % max_delta_U = [10; 1; 10]; % �������������ֵ
    % % ʩ�ӿ�������Լ��
    % opti.subject_to(-max_delta_U(1) <= delta_U(1, :) <= max_delta_U(1));
    % opti.subject_to(-max_delta_U(2) <= delta_U(2, :) <= max_delta_U(2));
    % opti.subject_to(-max_delta_U(3) <= delta_U(3, :) <= max_delta_U(3));
%%
% ����Լ������
opti.subject_to(X(:, 1) == X0);

opti.subject_to(  0 <= U(1, :)+Ref_path0(4,:) <= 14);
opti.subject_to( -8 <= U(2, :)+Ref_path0(5,:) <= 8);
opti.subject_to( -2 <= U(3, :)+Ref_path0(6,:) <= 2);


% �Ż���������
opti.minimize(J);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0; % 0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
opti.solver('ipopt', opts);


% ��ʼ��״̬���� x = [X - X_r; Y - Y_r; phi - phi_r]
x_Init = x_car(1:3, 1) - Ref_path_Init(1:3, 1);
% ���Ʊ��� U = [vx - vx_r; vy - vy_r; r - r_r]
% �����Ż������ĳ�ʼֵ
opti.set_value(X0, x_Init);
opti.set_value(Ref_path0, Ref_path_Init); 
opti.set_value(U_prev, u_prev_init); % ����ǰһʱ�̵Ŀ����źų�ʼֵ
sol = opti.solve();
uout = sol.value(U);
X_out = sol.value(X);
% ����״̬���ͱ仯��


predict_data(1, :) = predict_data(1, :) + X_out(1, 1:N);
predict_data(2, :) = predict_data(2, :) + X_out(2, 1:N);
predict_data(3, :) = predict_data(3, :) + X_out(3, 1:N);
u_car = uout + Ref_path_Init(4:6, :);
%��ӡ
% disp(u_car)

% ����ʵ�ʿ����� u_car = [vx; vy; r]
% Ӧ�ÿ�����

plot(x_car(1), x_car(2), '*');
hold on;
% plot(X_out(1, 1:N) + Ref_path_Init(1, :), X_out(2, 1:N) + Ref_path_Init(2, :), 'r');
plot(predict_data(1, :), predict_data(2, :), 'r');

% ��ӡ������Ϣ
sys(1) = u_car(1,1);  % ͬʱ sys ���浱ǰ�Ŀ���ֵ��sys[1] ��ϵͳ��������� Simulink �� S-function �ӿ�
sys(2) = atan(u_car(2,1) / (u_car(1,1) + eps));
sys(3) = u_car(3,1);
sys(4) = 6000;
if sys(3) < 0
    sys(2) = 0;
end


function h_val = h(obstacle_position,X, U, Refpath0)
    % ��������ϰ����� CBF
    % ������һ���ϰ��ȷ���������ϰ���֮��ľ������ĳ����ȫֵ
    
    % ���㳵����ǰλ�����ϰ���֮��ľ���
    distance = norm(X(1:2,:) + Refpath0(1:2,:) - obstacle_position);
    % ���� CBF ֵ
    h_val = distance ;

function initializePlot()
    % clf;
    % figure(1);
    [~, trajectory] = findNearestPoint([0, 0], 0, 0);
    plot(trajectory(:, 1), trajectory(:, 2), 'b');
    hold on;
    grid on;
    % ��Բ���� (0, 1) �뾶Ϊ 1 ��Բ
    theta = linspace(0, 2*pi, 100);
    x_circle = 2 * cos(theta) -  5;
    y_circle = 2 * sin(theta) - 39;
    plot(x_circle, y_circle, 'r');

    % % ��Բ���� (0, 1) �뾶Ϊ 1 ��Բ
    % theta = linspace(0, 2*pi, 100);
    % x_circle = 27 * cos(theta) -48;
    % y_circle = 27 * sin(theta) -46;
    % plot(x_circle, y_circle, 'b');
    %Ư�ƿɿ�Լ��

% ����sigmoid����
function y = sigmoid(x)
    y = 1 / (1 + exp(-x));
% ����tanh����

function drift_control_penalty= calculate_penalties(Ux_car, Uy_car, r_car)

    % ����softplus����
    penalty_drift = 0; % Ư��״̬�µĳͷ�ϵ��
    penalty_non_drift = 0; % ��Ư��״̬�µĳͷ�ϵ��
    drift_r_min = 0.2; % Ư��״̬�µĺ�����ٶ�����
    non_drift_beta_max = 0.2; % ��Ư��״̬�µĲ໬������
    non_drift_Uy_max = tan(non_drift_beta_max) * Ux_car; % ��Ư��״̬�µĺ����ٶ�����

    % Ư��״̬�µ���Լ��
    drift_condition = sigmoid(10 * (Uy_car^2 - non_drift_Uy_max^2));
    sign_same = sigmoid(20*Uy_car * r_car);
    
    drift_penalty = penalty_drift * drift_condition * (sigmoid(100 *(drift_r_min^2 - r_car^2))+10000*sign_same);

    % ��Ư��״̬�µ���Լ��
    non_drift_condition = sigmoid(10 * (drift_r_min^2 - r_car^2));
    sign_diff = sigmoid(-20*Uy_car * r_car);
    non_drift_penalty = penalty_non_drift * non_drift_condition *  (sigmoid(Uy_car^2)+10000*sign_diff);

    drift_control_penalty = drift_penalty + non_drift_penalty;
