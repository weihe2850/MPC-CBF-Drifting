function   pars = Refpath_parameters( Time )

%% Carsim_2020�汾 Class_C������  ����ϵͳ������300kW,6-spd,4:1 Ratio
%�켣�ο�״̬
    if Time >40
        pars.Ux_p1=12;
        pars.beta_p1=-0.5; 
        pars.r_p1=0.5;
        
        
        pars.radius_p1 = 25;           % m / s^2
        pars.start_theta_p1 =-pi/2;         
        pars.end_theta_p1 = 3*pi/2;        
        pars.number_p1 = 200;         
    else
        % pars.Ux_p1=10;
        % pars.beta_p1=-0.5; 
        % pars.r_p1=0.6;
        % pars.radius_p1 = 19;           % m / s^2
        pars.Ux_p1=10;
        pars.beta_p1=0; 
        pars.r_p1=0.6;
        pars.radius_p1 = 19;           % m / s^2

        pars.start_theta_p1 =-pi/2;         
        pars.end_theta_p1 = 3*pi/2;        
        pars.number_p1 = 200;      
    end

end
