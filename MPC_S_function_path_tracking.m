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
sizes.NumContStates = 0;	% ���������źŸ���
sizes.NumDiscStates = 0;	% ������ɢ�źŸ���
sizes.NumOutputs = 4;     	% ����źŸ���������ָ���Ҹ�������ź�
sizes.NumInputs = 10;		% �����źŸ���������ָPID�������ڵ�����
sizes.DirFeedthrough = 1;	% ����ϵͳֱ��ͨ������������һ��Ϊ1
sizes.NumSampleTimes = 1;	% ��Ҫ������ʱ�䣬һ��Ϊ1.

sys = simsizes(sizes);    
x0 = [];
str = [];
ts = [0.1,0];
% ts = [ ];
% �������
clf;
figure(1);
[~,trajectory]=findNearestPoint([0,0], 0 ,0);
plot(trajectory(:,1),trajectory(:,2),'b');

hold on;
grid on;
% ��Բ���� (-0.5, -25) �뾶Ϊ 1 ��Բ
theta = linspace(0, 2*pi, 100);
x_circle = 0 +1* cos(theta);
y_circle = 0 + 1*sin(theta);
plot(x_circle, y_circle, 'r');

function sys = mdlOutputs(t,x,u) 
%%

Time=u(7);
%��ʼ��
opti = casadi.Opti();
%Ԥ�ⲽ��
N=20;
Ts=0.1;
%     ��ʼ��ֵ �ο��켣Ref_path = [X_r;Y_r;phi_r;vx_r;vy_r;r_r];
Ref_path0 = opti.parameter(6, N);
X0 = opti.parameter(3, 1);
predict_data=zeros(2,N);
%     ״̬��X = [X-X_r;Y-Y_r;phi-phi_r];
%     ������U = [vx-vx_r;vy-vy_r;r-r_r];
%     �ο��켣ƥ��� =[X_r;Y_r;phi_r,vx_r;vy_r;r_r]
X = opti.variable(3, N + 1);
U = opti.variable(3, N);
delta_U = opti.variable(3, N );
%�����ݴ�״̬��
J = 0;
if Time<5
    Q_M = 1e01*([5 0 0;
                 0 5 0;
                 0 0 1]);
    %������Ȩ��
    R = 1e04*([1 0 0;
               0 5 0;
               0 0 1]);
        %ģ��Ԥ�ⲿ��
    for k=1:N    
        %���ڹ켣���ģ�ͣ�ͨ����ǰ������Ԥ�����һ�ڵ��״̬
        opti.subject_to( X(:,k+1)==vehicle_path_tracking_model( X(:,k),U(:,k),Ref_path0(:,k),Ts) ); 
        J=J+X(:,k)' * Q_M *X(:,k) + U(:, 1)' * R * U(:, 1);      
        opti.subject_to(delta_U(:, k) == 0);
    end

else
    Q_M = 1e03*([5 0 0;
                 0 5 0;
                 0 0 1]);
    %������Ȩ��
    R = 1e01*([1 0 0;
               0 500 0;
               0 0 1]);
    %ģ��Ԥ�ⲿ��
    for k=1:N    
        %���ڹ켣���ģ�ͣ�ͨ����ǰ������Ԥ�����һ�ڵ��״̬
        opti.subject_to( X(:,k+1)==vehicle_path_tracking_model( X(:,k),U(:,k),Ref_path0(:,k),Ts) );  
        % ���� CBF ��ȫԼ��
        % opti.subject_to(h(X(:, k), U(:, k) , Ref_path0(:,k)) >= 0);
        % J=J+X(:,k)' * Q_M *X(:,k) + U(:, 1)' * R * U(:, 1);    
        epsilon = 1e-6; % ����Ը�����Ҫ�������ֵ
        % ԭʼĿ�꺯��
        J = J + X(:,k)' * Q_M * X(:,k) + U(:, 1)' * R * U(:, 1);
        % ���� h �ӽ� 0 ʱ�ĳͷ�
        penalty = 1000000 / (h(X(:, k), U(:, k), Ref_path0(:,k))^2 + epsilon); % epsilon ��һ��С����������ֹ������
        J = J + penalty;    
    end

    % ���� delta_U
    opti.subject_to(delta_U(1, 1) == (U(1, 1) - u(8)) / Ts);
    opti.subject_to(delta_U(2, 1) == (U(2, 1) - u(9)) / Ts);
    opti.subject_to(delta_U(3, 1) == (U(3, 1) - u(10)) / Ts);
    for k = 1:N-1
        opti.subject_to(delta_U(:, k) == (U(:, k+1) - U(:, k)) / Ts);
    end
    %%

    max_delta_U = [200; 20; 20]; % �������仯�ʵ�������
    % ���ӿ������仯��Լ��
    opti.subject_to(-max_delta_U(1) <= delta_U(1, :) <= max_delta_U(1));
    opti.subject_to(-max_delta_U(2) <= delta_U(2, :) <= max_delta_U(2));
    opti.subject_to(-max_delta_U(3) <= delta_U(3, :) <= max_delta_U(3));
end
%%
%�߽�Լ������
opti.subject_to(X(:, 1) == X0);

opti.subject_to(  0 <= U(1, :)+Ref_path0(4,:) <= 14);
opti.subject_to( -6 <= U(2, :)+Ref_path0(5,:) <= 6);
opti.subject_to( -1 <= U(3, :)+Ref_path0(6,:) <= 1);



%�����Ż����������
opti.minimize(J);
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
opti.solver('ipopt',opts);
%% MPC���
%״̬��X = [X-X_r;Y-Y_r;phi-phi_r];
x_car=[0;0;0];
%���복����ǰ�켣״̬x_car = [X;Y;phi];
x_car(1)=u(1);
x_car(2)=u(2);
x_car(3)=u(3);
Ux=u(4);
beta=u(5);
r=u(6);
V=Ux/(cos(beta)+eps);

%���ɳ�ʼ�ο��켣
Ref_path_Init=zeros(6,N);
for i=1:N
%���������
    currentPosition=[x_car(1)+(i-1)*Ts*V*cos(x_car(3)+beta),x_car(2)+(i-1)*Ts*V*sin(x_car(3)+beta)];

    nearestPoint = findNearestPoint(currentPosition , x_car(3) , Time);
    %��ʼ�������ƥ���
    X_r=nearestPoint(1);
    Y_r=nearestPoint(2);
    theta_r=nearestPoint(3);  
    Ux_r=nearestPoint(4);
    Uy_r=nearestPoint(5);
    r_r=nearestPoint(6);  
    %%������ٵ�Ĳο�����״̬
    Ref_path_Init(1,i)=X_r;
    Ref_path_Init(2,i)=Y_r;
    Ref_path_Init(3,i)=theta_r;  
    Ref_path_Init(4,i)=Ux_r;
    Ref_path_Init(5,i)=Uy_r;
 
    Ref_path_Init(6,i)=r_r;  
    
    predict_data(1,i)=X_r;
    predict_data(2,i)=Y_r;
end
%    Ref_path_Init(5,1)=Ref_path_Init(5,N);
%    Ref_path_Init(6,1)=Ref_path_Init(6,N);
%��ʼ�Ż�״̬��x = [X-X_r;Y-Y_r;phi-phi_r];
x_Init = x_car(1:3,1)-Ref_path_Init(1:3,1);
%�Ż�������U = [vx-vx_r;vy-vy_r;r-r_r];

%����Ż�����  �õ����ſ�����
opti.set_value(X0, x_Init);
opti.set_value(Ref_path0, Ref_path_Init); 

sol = opti.solve();
uout=sol.value(U);

X_out=sol.value(X);
delta_U_out = sol.value(delta_U);


predict_data(1,:)=predict_data(1,:)+X_out(1,1:N);
predict_data(2,:)=predict_data(2,:)+X_out(2,1:N);
%����ʵ�ʿ�����u_car = [vx;vy;r];
%Ӧ�ÿ����� 
u_car=uout(:,1)+Ref_path_Init(4:6,1);
plot(x_car(1),x_car(2),'*');
hold on;
% plot(X_out(1,1:N)+Ref_path_Init(1,:),X_out(2,1:N)+Ref_path_Init(2,:),'r');
plot(predict_data(1,:),predict_data(2,:),'r');

% axis([-25 25 -25 25]);
% ��ӡ������Ϣ

sys(1) = u_car(1);  % ͬʱsys��ǰ������ֵ��sys[1]��)��ϵͳ�����������simulink��S-function������ϵ�����
sys(2) = atan(u_car(2)/(u_car(1)+eps)) ;
sys(3) = u_car(3);
sys(4) =6000;



function h_val = h(X, U ,Refpath0)
    % ���ﶨ����� CBF ����
    % ���磬������ϣ��ȷ���������ϰ���֮��ľ������ĳ����ֵ
    obstacle_position = [0; 0]; % �ϰ���λ��
    safe_distance = 1; % ��ȫ������ֵ
    
    % ���㳵����ǰλ�����ϰ���֮��ľ���
    distance = norm( X(1:2) + Refpath0(1:2) - obstacle_position);
    
    % ���� CBF ����
    h_val = distance - safe_distance;



