function   pars=car_parameters()

%% Carsim_2020版本 Class_C车参数  动力系统：后驱300kW,6-spd,4:1 Ratio
pars.g = 9.8;           % m / s^2
pars.m = 1650;          % kg
pars.Iz = 1536;         % kg / m^2
pars.a = 1.015;          % m
pars.b = 1.895;          % m

pars.B = 12.13;      % N / rad
pars.C = 1.451;      % N / rad
mu=1;
pars.mu = mu;         % dimensionless
pars.Fz = 6500;
pars.FzR = 6500;
pars.FzF = 6500;

pars.delta_max =35*pi/180;
    
pars.FxR_max = 6500*mu;
pars.L = 2.91; % m
pars.CaF = 12.13*1.451*mu*6500;      % N / rad
pars.CaR = 12.13*1.451*mu*6500;       % N / rad

% %% eq states  在give_states_eq模块中给定
% pars.Ux_eq      = 10;
% pars.beta_eq    = -30*pi/180;
% pars.r_eq       = 0.5;
% pars.FxR_eq     = 5200;

%% Controller parameters
% pars.K_Ux   = 1.2;
% pars.K_beta = 2.3;
% pars.K_r    = 4;
pars.K_Ux   = 1.2;
pars.K_beta = 5;
pars.K_r    =6;
% pars.K_Ux_normal = 1000;
% pars.K_r_normal    = 1;
end
