%% Animation 
function animate(z,pack,p,slow_mo)

    flag_movie = p.flag_movie;
    if flag_movie
        name = ['test.mp4'];
        vidfile = VideoWriter(name,'MPEG-4');
        open(vidfile);
    end

    L = p.L;
    l = 0.14;
    rot = @(th)[cos(th) -sin(th);sin(th) cos(th)];

    [t_,X_,U_] = unPackDecVar(z,pack);
    t = linspace(t_(1),t_(end),50);
    X = interp1(t_',X_',t')';
    U = interp1(t_',U_',t')';
    

    figure(1)
    set(gcf, 'Color', 'white')
    clf;
    hold on
     
    h_pc    = plot([0],[0],'r.','MarkerSize',30);   % Ball handle
    h_torso = plot([0],[0],'b-','LineWidth',4);     % Bar handle
    h_leg1  = plot([0],[0],'g-','LineWidth',2);     % left leg
    h_leg2  = plot([0],[0],'g-','LineWidth',2);     % right leg
    h_traj  = plot([0],[0],'k--');                  % Trajectory handle
    h_u1    = plot([0],[0],'r','LineWidth',1);      % Trust vector handle
    h_u2    = plot([0],[0],'r','LineWidth',1);      % Trust vector handle
    h_Fg    = plot([0],[0],'r','LineWidth',1);
    h_title = title('t=0.0s');                      % Title handle
    
    h_traj.XData = [];
    h_traj.YData = [];
    
    axis equal;
    axis([-0.8 0.2 -0.3 0.5])
    pf1 = [-L/2;0];
    pf2 = [L/2;0];
    len = length(t);
    for ii = 1:len
        % Get state at current time
        x = X(:,ii);        % x = [pc;th;dpc;dth]
        th = x(3);
        pc = [x(1);x(2)];
        ph1 = [x(1)-L/2*cos(th);x(2)-L/2*sin(th)];
        ph2 = [x(1)+L/2*cos(th);x(2)+L/2*sin(th)];
        if t(ii) > p.Tds
            pf2 = ph2 + [0;-0.18];
        end
        
        % torso
        h_torso.XData = [ph1(1) ph2(1)];
        h_torso.YData = [ph1(2) ph2(2)];
        h_pc.XData = x(1);
        h_pc.YData = x(2);
        h_traj.XData(end+1) = x(1);
        h_traj.YData(end+1) = x(2);
        
        % leg
        pk1 = fcn_get_pk(ph1,pf1,p);
        pk2 = fcn_get_pk(ph2,pf2,p);
        h_leg1.XData = [ph1(1) pk1(1) pf1(1)];
        h_leg1.YData = [ph1(2) pk1(2) pf1(2)];
        h_leg2.XData = [ph2(1) pk2(1) pf2(1)];
        h_leg2.YData = [ph2(2) pk2(2) pf2(2)];
        
        % ground
        plot([-1 1],[0,0],'k-','linewidth',2)
        quiver(0,0,0.1,0,'r-','linewidth',2)
        quiver(0,0,0,0.1,'b-','linewidth',2)
        
        % forces
        scale = 1/150;
        u1 = U(1:2,ii) * scale;
        u2 = U(3:4,ii) * scale;
        Fg = [0;-p.m*p.g] * scale;
        h_u1.XData = [pf1(1) pf1(1)+u1(1)];
        h_u1.YData = [pf1(2) pf1(2)+u1(2)];
        h_u2.XData = [pf2(1) pf2(1)+u2(1)];
        h_u2.YData = [pf2(2) pf2(2)+u2(2)];
        h_Fg.XData = [pc(1) pc(1)+Fg(1)];
        h_Fg.YData = [pc(2) pc(2)+Fg(2)];
        
        h_title.String = sprintf('t=%.2f\n',t(ii));
        
        drawnow;
        if flag_movie
            writeVideo(vidfile, getframe(gcf));
        end
        pause(slow_mo*1e-3)
    end
    if flag_movie
        close(vidfile);
    end
end