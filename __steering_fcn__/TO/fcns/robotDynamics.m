function dXdt = robotDynamics(X,u,p)
    
    % X = [x;z;dx;dz;q1;q2;dq1;dq2]
    x = X(1,:);
    z = X(2,:);
    dx = X(3,:);
    dz = X(4,:);

    % u = [ux;uz]
    ux = u(1,:);
    uz = u(2,:);
    
    % dynamics
    ddx = 1/p.mass * ux;
    ddz = 1/p.mass * uz - p.g;
    
    dXdt = [dx;dz;ddx;ddz];
    
end