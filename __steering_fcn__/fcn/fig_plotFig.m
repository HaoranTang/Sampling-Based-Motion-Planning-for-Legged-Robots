function fig_plotFig(val,Nstep,p)

% ---- parameters -----
c_red = [0.85 0.325 0.098];
c_blue = [0 0.447 0.741];
c_orange = [255 179 0]/255;
% ---------------------

mass = p.mass;
g = p.g;

Fx_var = val.Fx_var;
Fz_var = val.Fz_var;
Tst = val.Tst;
Tair = val.Tair;
Tair = [0;Tair];
xval = val.x;
zval = val.z;
dxval = val.dx;
dzval = val.dz;
xTD = val.xTD;
zTD = val.zTD;



fig_F = figure('position',[200 200 300*Nstep 500]);
% fig_tau = figure('position',[200 200 300*Nstep 600]);

for istep = 1:Nstep
    dx0 = dxval(:,1);
    dz0 = dzval(:,1);
    x0 = xval(:,1);
    z0 = zval(:,1);
    
    % coefficient through the multiple steps
    Fx_co(istep,:) = [0 Fx_var(istep,:) 0];
    Fz_co(istep,:) = [mass*g Fz_var(istep,:) 0];
    dx_co(istep,:) = bz_int(Fx_co(istep,:)/mass,dx0(istep),Tst(istep));
    dz_co(istep,:) = bz_int(Fz_co(istep,:)/mass - g,dz0(istep),Tst(istep));
    x_co(istep,:) = bz_int(dx_co(istep,:),x0(istep),Tst(istep));
    z_co(istep,:) = bz_int(dz_co(istep,:),z0(istep),Tst(istep));
    
    s = linspace(0,1,101);
    
    Fx = polyval_bz(Fx_co(istep,:),s);
    Fz = polyval_bz(Fz_co(istep,:),s);
    dx = polyval_bz(dx_co(istep,:),s);
    dz = polyval_bz(dz_co(istep,:),s);
    x = polyval_bz(x_co(istep,:),s);
    z = polyval_bz(z_co(istep,:),s);
    
    % GRF
    figure(fig_F);
    subplot(2,1,1)
    hold on;grid on;box on;
    t = Tair(istep) + Tst(istep) * s;
    plot(t,Fx,'color',c_blue,'linewidth',2)
    plot(t,Fz,'color',c_red,'linewidth',2)
    legend('Fx','Fz','location','north')
    title('F')
    
    
    for ii = 1:length(s)
        r = [x(ii);z(ii)];
        J = getJacobian(r);
        tau(ii,:) = (J' * [Fx(ii);Fz(ii)])';
    end
    
    % torque
    figure(fig_F);
    subplot(2,1,2)
    hold on;grid on;box on;
    plot(t,tau(:,1),'color',c_blue,'linewidth',2)
    plot(t,tau(:,2),'color',c_red,'linewidth',2)
    legend('tau_h','tau_k','location','north')
    title('tau')
    

    
    
    
    


end







