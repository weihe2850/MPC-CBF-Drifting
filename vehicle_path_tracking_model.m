function x_update =  vehicle_path_tracking_model(x,u,Ref_path,Ts)%%预测模型
    X_r=Ref_path(1);
    Y_r=Ref_path(2);
    phi_r=Ref_path(3);
    vx_r=Ref_path(4);
    vy_r=Ref_path(5);
    r_r=Ref_path(6);
    %轨迹误差模型
    A=[0 0 -vx_r*sin(phi_r)-vy_r*cos(phi_r);
       0 0  vx_r*cos(phi_r)-vy_r*sin(phi_r);
       0 0               0            ];   
    B=[cos(phi_r) -sin(phi_r) 0;
       sin(phi_r)  cos(phi_r) 0;
        0             0       1];   
    C=eye(3); 
%     ksi=[X-X_r;Y-Y_r;phi-phi_r];
%     u = [vx-vx_r;vy-vy_r;r-r_r];
    x_dot=A * x +B * u;
    x_update=x+x_dot*Ts;                                                  

end

