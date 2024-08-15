function pars = Refpath_parameters(Time)

    %% Carsim_2020�汾 Class_C������  ����ϵͳ������300kW,6-spd,4:1 Ratio
    %�켣�ο�״̬
    off_value = 100;
    Time1 = 8.3 +off_value;
    Time2 = 9.3 +off_value;
    Time3 = 10.4 +off_value;
    if 0 <= Time && Time < Time1
        pars.Ux_p1 = 12;
        pars.beta_p1 = -0.5;
        pars.r_p1 = 0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -5;
        pars.center_Y = -13;
    elseif Time1 <= Time && Time < Time2
        pars.Ux_p1 = 12;
        pars.beta_p1 = 0;
        pars.r_p1 = -0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -5;
        pars.center_Y = -16;
    elseif Time2 <= Time && Time < Time3
        pars.Ux_p1 = 12;
        pars.beta_p1 = 0;
        pars.r_p1 = 0.5;
    
        pars.radius_p1 = 27;           % m / s^2
        pars.start_theta_p1 = -pi/2;
        pars.end_theta_p1 = 3*pi/2;
        pars.number_p1 = 200;
        pars.center_X = -5;
        pars.center_Y = -13;
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
