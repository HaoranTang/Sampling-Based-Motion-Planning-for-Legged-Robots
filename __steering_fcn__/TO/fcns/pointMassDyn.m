function dXdt = pointMassDyn(X,u,p)
    
    % X = [x;y;dx;dy]
    dx = X(3,:);
    dy = X(4,:);
    
    % u = [ux;uy]
    ux = u(1,:);
    uy = u(2,:);
    
    % dynamics
    [ddx,ddy] = autoGen_pointMassDynamics(ux,uy,p.m,p.g);
    
    dXdt = [dx;dy;ddx;ddy];
    
end