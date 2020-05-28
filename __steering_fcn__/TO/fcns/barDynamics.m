function dXdt = barDynamics(X,u,p)
    
    % X = [x;y;th;dx;dy;dth]
    th = X(3,:);
    dq = X(4:6,:);

    % u = [u1x;u1y;u2x;u2y]
    u1x = u(1,:);
    u1y = u(2,:);
    u2x = u(3,:);
    u2y = u(4,:);
    
    % dynamics
    [ddx,ddy,ddth] = autoGen_barDynamics(th,u1x,u1y,u2x,u2y,...
                                        p.m,p.J,p.g,p.L);
    
    dXdt = [dq;ddx;ddy;ddth];
    
end