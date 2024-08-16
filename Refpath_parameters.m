function pars = Refpath_parameters(Time)

    %% Carsim_2020版本 Class_C车参数  动力系统：后驱300kW,6-spd,4:1 Ratio
    %轨迹参考状态
    if 0 <= Time && Time < 20
        pars.Ux_p1 = 12;
        pars.beta_p1 = -0.5;
        pars.r_p1 = 0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -5;
        pars.center_Y = -13;
    elseif 20 <= Time && Time < 400
        pars.Ux_p1 = 12;
        pars.beta_p1 = 0.5;
        pars.r_p1 = -0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -48;
        pars.center_Y = -46;
    else
        pars.Ux_p1 = 12;
        pars.beta_p1 = -0.5;
        pars.r_p1 = 0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -5;
        pars.center_Y = -13;
    end
    
end
