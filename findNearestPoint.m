function [nearestPoint, trajectory] = findNearestPoint(currentPosition, currentTheta , Time )
    % ��ʼ���켣����
    pars = Refpath_parameters(Time);
    
    % ���ɹ켣��
    % [x_p1, y_p1] = generateTrajectory(pars.radius_p1, pars.start_theta_p1, pars.end_theta_p1, pars.number_p1, 3, 7);
    [x_p1, y_p1] = generateTrajectory(pars.radius_p1, pars.start_theta_p1, pars.end_theta_p1, pars.number_p1, -pars.center_X, -pars.center_Y);
    % �ϲ��켣��
    x = x_p1;
    y = y_p1;
    trajectory = [x', y'];
    
    % ���������
    currentPosition = repmat(currentPosition, length(x), 1);
    distances = vecnorm(trajectory - currentPosition, 2, 2);
    [~, nearestPointIndex] = min(distances);
    
    % �������߷���õ��ο������
    tangent_angle = calculateTangentAngle(trajectory, nearestPointIndex);
    
    % ͳһ�켣�ο��ǶȺͳ����Ƕ�
    angle_difference = mod(currentTheta - tangent_angle + pi, 2*pi) - pi;
    if abs(angle_difference) > pi/2
        angle_difference = angle_difference - pi * sign(angle_difference);
    end
    
    % ��ȡ�ο�״̬
    [Ux_r, Uy_r, r_r] = getReferenceState(pars, nearestPointIndex);
    
    % �����������Ϣ
    X_r = trajectory(nearestPointIndex, 1);
    Y_r = trajectory(nearestPointIndex, 2);
    theta_r = currentTheta - angle_difference;
    nearestPoint = [X_r, Y_r, theta_r, Ux_r, Uy_r, r_r]';
end


function [x, y] = generateTrajectory(radius, start_theta, end_theta, number, center_X, center_Y)
    theta = My_linspace(start_theta, end_theta, number);
    x = radius * cos(theta) - center_X;
    y = radius * sin(theta) - center_Y;
end

function tangent_angle = calculateTangentAngle(trajectory, index)
    if index > 1 && index < length(trajectory)
        tangent_angle = atan2(trajectory(index+1, 2) - trajectory(index-1, 2), trajectory(index+1, 1) - trajectory(index-1, 1));
    elseif index == 1
        tangent_angle = atan2(trajectory(index+1, 2) - trajectory(index, 2), trajectory(index+1, 1) - trajectory(index, 1));
    else
        tangent_angle = atan2(trajectory(index, 2) - trajectory(index-1, 2), trajectory(index, 1) - trajectory(index-1, 1));
    end
end

function [Ux_r, Uy_r, r_r] = getReferenceState(pars, index)
    Ux_r = pars.Ux_p1;
    Uy_r = tan(pars.beta_p1) * Ux_r;
    r_r  = pars.r_p1;
end

function linspace_data = My_linspace(start_theta, end_theta, number)
    if start_theta <= end_theta
        linspace_data = linspace(start_theta, end_theta, number);
    else
        linspace_data = linspace(end_theta, start_theta, number);
        linspace_data = flip(linspace_data);
    end
end