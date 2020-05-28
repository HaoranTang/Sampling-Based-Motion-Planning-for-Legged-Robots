function fig_plot2D(infObs,grid,val,Nstep,M_st,M_air,p)
% infObs: [pObs;slope], information of obstacles
% grid: grid matrix
% val: value for the optimization variables
% Nstep: number of steps
% M_st: number of time knot in stance phase
% M_air: number of time knot in aerial phase
% p: parameter structure
% 


% ---- parameters -----
c_red = [0.85 0.325 0.098];
c_blue = [0 0.447 0.741];
c_orange = [255 179 0]/255;
% ---------------------

g = p.g;
flag_terrain = p.fterrain;

Fx_var = val.Fx_var;
Fz_var = val.Fz_var;
x = val.x;
z = val.z;
dx = val.dx;
dz = val.dz;
xTD = val.xTD;
zTD = val.zTD;
Tair = val.Tair;
Tst = val.Tst;
bval = val.b;

%%
x_st_list = cell(Nstep,1);  z_st_list = cell(Nstep,1);
x_fl_list = cell(Nstep,1);  z_fl_list = cell(Nstep,1);
dx_st_list = cell(Nstep,1); dz_st_list = cell(Nstep,1);
dx_fl_list = cell(Nstep,1); dz_fl_list = cell(Nstep,1);
t_st_list = cell(Nstep,1);  t_fl_list = cell(Nstep,1);

xlist = [];     zlist = [];
dxlist = [];    dzlist = [];
Fxlist = [];    Fzlist = [];
Tairlist = [];
tlist = [];
dqlist = [];

xTDval_sol = [];
zTDval_sol = [];

for istep = 1:Nstep

    t_st = linspace(0,Tst(istep),M_st);     % stance
    t_air = linspace(0,Tair(istep),M_air);  % aerial
    
    x_st_list{istep} = xTD(istep) + x(istep,:);
    z_st_list{istep} = zTD(istep) + z(istep,:);
    dx_st_list{istep} = dx(istep,:);
    dz_st_list{istep} = dz(istep,:);

    
    x_fl_list{istep} = xTD(istep) + x(istep,end) + dx(istep,end)*t_air;
    z_fl_list{istep} = zTD(istep) + z(istep,end) + dz(istep,end)*t_air - 1/2*g*t_air.^2;
    if istep == 1
        t_st_list{istep} = t_st;
        t_fl_list{istep} = t_st(end) + (istep-1)*(Tst(istep) + Tair(istep)) + t_air;
    else
        t_st_list{istep} = t_st + (istep-1)*(Tst(istep) + Tair(istep-1));
        t_fl_list{istep} = t_st(end) + (istep-1)*(Tst(istep-1) + Tair(istep-1)) + t_air;
    end
    n_st = length(x_st_list{istep}(1:end-1));
    n_fl = length(x_fl_list{istep}(1:end-1));
    
    xlist = [xlist,x_st_list{istep}(1:end-1),x_fl_list{istep}(1:end-1)];
    zlist = [zlist,z_st_list{istep}(1:end-1),z_fl_list{istep}(1:end-1)];
    
    tlist = [tlist,t_st_list{istep}(1:end-1),t_fl_list{istep}(1:end-1)];
    
    xTDval_sol = [xTDval_sol,xTD(istep)*ones(1,n_st), xTD(istep)+x_fl_list{istep}(1:end-1)-linspace(x_fl_list{istep}(1),x_fl_list{istep}(end-1)+xTD(istep)-xTD(istep+1),n_fl)];
    zTDval_sol = [zTDval_sol,zTD(istep)*ones(1,n_st), zTD(istep)+z_fl_list{istep}(1:end-1)-linspace(z_fl_list{istep}(1),z_fl_list{istep}(end-1)+zTD(istep)-zTD(istep+1),n_fl)];
    
    %%
    hold on;axis equal
    % ---- plot ground ---
    fig_plotObs(infObs);
    % --------------------
    
    % COM trajectory in stance
    plot(x_st_list{istep},z_st_list{istep},'color',c_red,'linestyle','-',...
                                           'marker','*','linewidth',1.5,...
                                           'markersize',4);
    % COM trajectory in flight
    plot(x_fl_list{istep},z_fl_list{istep},'color',c_blue,'linewidth',1.5);
    
    if flag_terrain == 2
        % goal region
        fill([0.2 0.4 0.4 0.2],[0.6 0.6 0.66 0.66],c_orange,'LineStyle','none')
        % marker
        plot(xTD(1),zTD(1),'rx','markersize',10,'linewidth',2)
        plot(xTD(2),zTD(2),'r^','markersize',8,'linewidth',2)
        plot(xTD(3),zTD(3),'ro','markersize',8,'linewidth',2)
    %     
    %     lgd = legend('Ground','Stance','Flight','Goal','Start','Step','Goal','location','southeast');
    %     lgd.FontSize = 12;
    end
    
    set(gca,'fontsize',12)
    xlabel('x [m]','fontsize',16)
    ylabel('z [m]','fontsize',16)
    
%     axis([-0.6 2.5 0 1])
    
    % grid
    color = 'b';
    grid_plot = grid;
    for i_t = 1 : M_st
        i_g = find(bval(i_t,:,istep)>0.1);
        temp = grid_plot{i_g} + [xTD(istep)*ones(3,1),zTD(istep)*ones(3,1)];
        patch(temp(:,1),temp(:,2),color);
        alpha(0.3);
    end
    for i_g = 1:length(grid_plot)
        grid_plot{i_g} = grid_plot{i_g} + [xTD(istep)*ones(3,1),zTD(istep)*ones(3,1)];
    end
    fig_plotGrid(grid_plot,'kx')

end








