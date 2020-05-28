% single leg MICP motion planning
function [bool_success] = fcn_steer_v1
% bool_success
% Xin = [qin;dqin]
% Xout = [qout;dqout]

Xin = [0;0];
Xout = [0;0];

% ---- parameters -----
p = getParams();
Nstep = 1;                  % number of steps

mass = p.mass;
g = p.g;
Nr = p.Nr;                     % number of nodes in r direction
Nth = p.Nth;                    % number of nodes in th direction
M_st = p.M_st;                   % number of sampling points in stance phase
Tst = 0.15*ones(Nstep,1);   % stance time
bigM = 100;
Tairmin = 0.1;              % max air time
Tairmax = 0.7;              % min air time
dxmin = -4;
dxmax = 4;
dzmin = 0;
dzmax = 4;
dqmin = [dxmin;dzmin];
dqmax = [dxmax;dzmax];


%% obstacle 
% pt1 = [-0.5 0 0]';
% pt2 = [1 0 0]';
% obstacle = [pt1 pt2];
% p.obstacle = obstacle;
% p_obs = obstacle(1:2,:); 
% th_obs = obstacle(3,:);
% Nobs = size(obstacle,2) - 1;
% fig_plotObs(obstacle);          % plot obstacle

%% Grid and FWP
Grid = fcn_init_grid(Nr,Nth);

fprintf('Grid generation complete!\n')

FWP = get_FWP(Grid);
fprintf('FWP generation complete!\n')

save('grid_FWP.mat','Grid','FWP')

% load('grid_FWP.mat')
fprintf('Grid and FWP loaded!\n')
% fig_plotGrid(Grid,[0;0]);             % plot grid

%% Define variables
Ngrid = length(Grid);

% binary variables
b_WS = binvar(M_st,Ngrid,Nstep);    % binvar for workspace
% b_obs = binvar(Nobs,Nstep);         % binvar for obstacle

% initial condition
% X0 = sdpvar(4,Nstep,'full');
X0 = Xin;

% force profile coefficients
N_co = 4;
Fx_var = sdpvar(Nstep,N_co,'full');
Fz_var = sdpvar(Nstep,N_co,'full');
Fx_co = sdpvar(Nstep,N_co+2,'full');
Fz_co = sdpvar(Nstep,N_co+2,'full');
Fx = sdpvar(Nstep,M_st,'full');
Fz = sdpvar(Nstep,M_st,'full');

ddx_co = sdpvar(Nstep,N_co+2,'full');
ddz_co = sdpvar(Nstep,N_co+2,'full');
dx_co = sdpvar(Nstep,N_co+3,'full');
dz_co = sdpvar(Nstep,N_co+3,'full');
x_co = sdpvar(Nstep,N_co+4,'full');
z_co = sdpvar(Nstep,N_co+4,'full');

ddx = sdpvar(Nstep,M_st,'full');
ddz = sdpvar(Nstep,M_st,'full');
dx = sdpvar(Nstep,M_st,'full');
dz = sdpvar(Nstep,M_st,'full');
x = sdpvar(Nstep,M_st,'full');
z = sdpvar(Nstep,M_st,'full');

for istep = 1:Nstep
    x0 = X0(1,istep);
    z0 = X0(2,istep);
    dx0 = X0(3,istep);
    dz0 = X0(4,istep);
    
    % B-coefficient
    Fx_co(istep,:) = [0 Fx_var(istep,:) 0];
    Fz_co(istep,:) = [0 Fz_var(istep,:) 0];
    
    ddx_co(istep,:) = Fx_co(istep,:) / mass;
    ddz_co(istep,:) = Fz_co(istep,:) / mass - g;
    dx_co(istep,:) = bz_int(ddx_co(istep,:),dx0,Tst(istep));
    dz_co(istep,:) = bz_int(ddz_co(istep,:),dz0,Tst(istep));
    x_co(istep,:) = bz_int(dx_co(istep,:),x0,Tst(istep));
    z_co(istep,:) = bz_int(dz_co(istep,:),z0,Tst(istep));
    
    % time and phase vectors
    t_vec(istep,:) = linspace(0,Tst(istep),M_st);
    s_vec = t_vec(istep,:)/Tst(istep);
    
    % trajectories
    Fx(istep,:) = polyval_bz(Fx_co(istep,:),s_vec);
    Fz(istep,:) = polyval_bz(Fz_co(istep,:),s_vec);
    ddx(istep,:) = polyval_bz(ddx_co(istep,:),s_vec);
    ddz(istep,:) = polyval_bz(ddz_co(istep,:),s_vec);
    dx(istep,:) = polyval_bz(dx_co(istep,:),s_vec);
    dz(istep,:) = polyval_bz(dz_co(istep,:),s_vec);
    x(istep,:) = polyval_bz(x_co(istep,:),s_vec);
    z(istep,:) = polyval_bz(z_co(istep,:),s_vec);
    
    wrench{istep} = [Fx(istep,:);Fz(istep,:)];
    dq{istep} = [dx(istep,:);dz(istep,:)];
    q{istep} = [x(istep,:);z(istep,:)];
    
end

% AUXILIARY variables
qTD     = sdpvar(2,Nstep+1,'full');    % touchdown q
qTO     = sdpvar(2,Nstep,'full');
dqTO    = sdpvar(2,Nstep,'full');
dqTair  = sdpvar(2,Nstep,'full');      % bilinear slack variable
qend    = sdpvar(2,1);                 % end q
dqend   = sdpvar(2,1);                 % end dq
Tair    = sdpvar(1,Nstep,'full');      % aerial time
Tair2   = sdpvar(1,Nstep,'full');      % Tair^2

fprintf('variable definition finished!\n')

%% Constraints
F = [];
% initial condition
F = [F;[-0.15;0.05] <= X0(1:2,1) <= [0.15;0.2]];    % initial position box
F = [F;X0(3:4,1) == [0;0]];
F = [F;qTD(:,1) == qTD1];

for istep = 1:Nstep
    % temporal uniqueness
    F = [F;(sum(b_WS(:,:,istep),2) == 1):'sum_b_WS'];
    
    % terrain
%     F = [F;sum(b_obs(:,istep)) == 1];

    % Mixed Integer Torque Constraints
    for i_t = 1:M_st
        for i_g = 1:Ngrid
            A_geo = Grid{i_g}.A;
            b_geo = Grid{i_g}.b;
            q_temp = q{istep}(:,i_t);
            bigM_binary = bigM * (1-b_WS(i_t,i_g,istep));
            F = [F;(A_geo*q_temp-b_geo <= bigM_binary):'[x;z] within polygon'];
        end
    end
    for i_t = 1:M_st
        for i_g = 1:Ngrid
            A_FWP = FWP{i_g}.A;
            b_FWP = FWP{i_g}.b;
            wrench_temp = wrench{istep}(:,i_t);
            bigM_binary = bigM * (1-b_WS(i_t,i_g,istep));
            F = [F;(A_FWP*wrench_temp-b_FWP <= bigM_binary):'MI wrench cst'];
        end
    end
    
    % continuity constraints for aerial phase
    F = [F;(qTO(:,istep) == qTD(:,istep) + q{istep}(:,end)):'TD-TO continuity position'];
    F = [F;(dqTO(:,istep) == dq{istep}(:,end)):'TD-TO continuity velocity'];
    
    if istep < Nstep
        % position level
        F = [F;(qTD(:,istep+1)+q{istep+1}(:,1) == qTO(:,istep) + dqTair(:,istep) + [0;-1/2*p.g*Tair2(istep)]):'q-continuity'];
        % velocity level
        F = [F;(dq{istep+1}(:,1) == dqTO(:,istep) + [0;-p.g*Tair(istep)]):'dq-continuity'];
    else
        % position level
        F = [F;(qTD(:,istep+1) + qend == qTO(:,istep) + dqTair(:,istep) + [0;-1/2*p.g*Tair2(istep)]):'q-continuity'];
        % velocity level
        F = [F;(dqend == dqTO(:,istep) + [0;-p.g*Tair(istep)]):'dq-continuity th'];
    end
    
    % McCormick Envelope for bilinear variable approximation
    % All of the variables in the var list should be defined individually
    F_binlin = cst_Mccomick([dqTO(1,istep);Tair(istep);dqTair(1,istep)],[6,5],[dqmin(1);dqmax(1)],[Tairmin;Tairmax]);
    F = [F;F_binlin:'dxTair = dx_end*Tair'];
    F_binlin = cst_Mccomick([dqTO(2,istep);Tair(istep);dqTair(2,istep)],[4,5],[dqmin(2);dqmax(2)],[Tairmin;Tairmax]);
    F = [F;F_binlin:'dzTair = dz_end*Tair'];
    
    % Tair2
    F_quad = cst_quadratic_approx(Tair(istep),Tair2(istep),7,[Tairmin;Tairmax]);
    F = [F,F_quad:'Tair2 == Tair^2'];

%     for i_obs = 1:Nobs
%         % qTD on the obstacle
%         bigM_binary = bigM * (1-b_obs(i_obs,istep));
%         y_obs = p_obs(2,i_obs) + tan(th_obs(i_obs)) * (qTD(1,istep+1) - p_obs(1,i_obs));
%         F = [F;p_obs(1,i_obs) + p.eMargin - bigM_binary <= qTD(1,istep+1) <= p_obs(1,i_obs+1) - p.eMargin + bigM_binary];
%         F = [F;y_obs - bigM_binary <= qTD(2,istep+1) <= y_obs + bigM_binary];
%     end
end

% final touchdown state
F = [F;-0.15 <= qend(1) <= 0.15];
F = [F;0.1 <= qend(2) <= 0.2];     % height constraint

% goal
F = [F; qTD(:,end) == qTD2];

fprintf('constraint definition finished!\n')

%% Optimize!
% obj = -qTD(1,end);
obj = [];
ops = sdpsettings('solver','gurobi','debug',1,'verbose',1);
sol = optimize(F,obj,ops);

if sol.problem == 0
    bool_success = 1;
    
    val.t = t_vec;
    val.X0 = value(X0);
    val.q = get_value(q,Nstep);
    val.dq = get_value(dq,Nstep);
    val.qTD = value(qTD);
    val.Tst = Tst;
    val.Tair = value(Tair);
    val.qend = value(qend);
    val.dqend = value(dqend);
    val.b_WS = value(b_WS);
    
    [px,pz,vx,vz,Ta] = fcn_get_output(val);

%     fig_plot2D(val);
%     
%     fig_plotGrid(Grid,val.qTD(:,end));
else
    bool_success = 0;
    
end

end

function [px,pz,vx,vz,Ta] = fcn_get_output(val)
    px = val.q(1,end);
    pz = val.q(2,end);
    vx = val.dq(1,end);
    vz = val.dq(2,end);
    Ta = val.Tair;
    

end

function [Grid] = fcn_init_grid(Nr,Nth)

rSpan = linspace(0.26,0.05,Nr);
thMin = -pi*0.45 + pi/2;
thMax = pi*0.45 + pi/2;
r_mat = repmat(rSpan',1,Nth);
th_mat = get_th_mat(rSpan,Nth,thMax,thMin);

X_mat = r_mat.*cos(th_mat);
Z_mat = r_mat.*sin(th_mat);

Grid = cell(Nr-1,2*(Nth-1));
for i_r = 1:Nr-1
    for i_th = 1:Nth-1
        Grid{i_r,2*i_th-1} = [X_mat(i_r,i_th),Z_mat(i_r,i_th);
                                X_mat(i_r,i_th+1),Z_mat(i_r,i_th+1);
                                X_mat(i_r+1,i_th),Z_mat(i_r+1,i_th)];
        Grid{i_r,2*i_th-1} = Grid{i_r,2*i_th-1};
                            
        Grid{i_r,2*i_th  } = [X_mat(i_r+1,i_th),Z_mat(i_r+1,i_th);
                                X_mat(i_r+1,i_th+1),Z_mat(i_r+1,i_th+1);
                                X_mat(i_r,i_th+1),Z_mat(i_r,i_th+1)];
        Grid{i_r,2*i_th  } = (Grid{i_r,2*i_th});
    end
end

Grid = Grid';
Grid = Grid(:);

Ngrid = length(Grid);
for i_g = 1:Ngrid
    Grid{i_g} = Polyhedron(Grid{i_g});
    Grid{i_g}.minHRep;
    Grid{i_g}.minVRep;
end

end

function th_mat = get_th_mat(rSpan,Nth,th_Max,thMin)

Nr = length(rSpan);
th_mat = zeros(Nr,Nth);
for ii = 1:Nr
%     thMax = min(th_Max,find_th_max(rSpan(ii)));
    thMax = th_Max;
    th_mat(ii,:) = linspace(thMax,thMin,Nth);
end
end

function th = find_th_max(r)

l = 0.15;
slope = 0;

syms x y
eqns = [(y + l*sin(slope))^2 + (x + l*cos(slope))^2 == l^2, x^2 + y^2 == r^2];
vars = [x,y];
[solx,soly] = solve(eqns,vars);

xx = solx(1);
yy = vpa(soly(2));

th = atan2(-xx,yy) + pi/2;
th = vpa(th,5);

end

function p = getParams()

L = 0.3;
p.L = L;
p.l1 = 0.14;
p.l2 = 0.14;
p.mass = 2;
p.g = 9.81;
p.mu = 1;
GR = 23.3594;               % gear ratio
p.umax = 0.42*GR;             % torque limit
p.eMargin = 0.1;        % margin to the edge of obstacle
p.scale = 5e-4;

p.Nr = 4;                     % number of nodes in r direction
p.Nth = 6;                    % number of nodes in th direction
p.M_st = 8;                   % number of sampling points in stance phase
p.M_air = 21;                 % number of sampling points in aerial phase

p.c_red = [0.85 0.325 0.098];
p.c_blue = [0 0.447 0.741];
p.c_orange = [255 179 0]/255;

end

function terrain = fig_plotObs(infObs,color)
    if nargin < 2
        color = 'black';
    end
    hold on;grid on;box on;
    % infObs: [pObs;...
    %          slope]
    N = size(infObs,2)-1;
    if N == 1
        plot(infObs(1,:),infObs(2,:),'k','linewidth',2)
    else
        terrain = zeros(2,2*N);
        for ii = 1:N
            terrain(:,2*ii-1) = infObs(1:2,ii);
            terrain(:,2*ii) = ...
               [infObs(1,ii+1);...
                infObs(2,ii) + tan(infObs(3,ii))*(infObs(1,ii+1)-infObs(1,ii))];
        end
        plot(terrain(1,:),terrain(2,:),'k','linewidth',2,'color',color);
        axis equal
    end
end

function fig_plotGrid(Grid,Ob)

hold on;box on;grid on;axis equal;

Ngrid = length(Grid);
c_gray = [211 211 211]/255;
c_blue = [0 0.447 0.741];

for i_g = 1:Ngrid
    g = Grid{i_g}.V + repmat(Ob(:)',[3,1]);
    plot(g([1:3,1],1),g([1:3,1],2),'color',c_blue,'linewidth',1)
end

end

function J = getJacobian(pt)
p = getParams;
l1 = p.l1;
l2 = p.l2;

th = atan2(pt(2),pt(1));
r = norm(pt,2);
cos_alpha = (l1^2+r^2-l2^2)/(2*l1*r);
alpha = acos(cos_alpha);

cos_beta = (l1^2+l2^2-r^2)/(2*l1*l2);
beta = acos(cos_beta);

q1 = -(pi - th + alpha);
q2 = pi - beta;

J = fcn_J([q1,q2],[l1,l2]);

end

function [J] = fcn_J(q,params)

  J(1,1)=- params(2)*sin(q(1) + q(2)) - params(1)*sin(q(1));
  J(1,2)=-params(2)*sin(q(1) + q(2));
  J(2,1)=params(2)*cos(q(1) + q(2)) + params(1)*cos(q(1));
  J(2,2)=params(2)*cos(q(1) + q(2));
  J = -J;
  
end

function alpha_int = bz_int(alpha,x0,s_max)

if nargin == 2
    s_max = 1;
end

[n,m] = size(alpha);        % make sure alpha is a row vector
if n > m
    alpha = alpha';
end

M = length(alpha);
AA = zeros(M+1,M+1);

for ii = 1:M
    AA(ii,ii:ii+1) = [-1 1];
end

AA = M/s_max*AA;
AA(M+1,1) = 1;


alpha_int = (AA\[alpha x0]')';

end

function FWP = get_FWP(Grid)

Ngrid = length(Grid);
FWP = cell(Ngrid,1);
for i_g = 1:Ngrid
    gg = Grid{i_g}.V;
    J1 = getJacobian(gg(1,:));
    J2 = getJacobian(gg(2,:));
    J3 = getJacobian(gg(3,:)); 

    fwp{1} = get_FWP_oneLeg(J1);
    fwp{2} = get_FWP_oneLeg(J2);
    fwp{3} = get_FWP_oneLeg(J3);

    A = [fwp{1}.A;fwp{2}.A;fwp{3}.A];
    b = [fwp{1}.b;fwp{2}.b;fwp{3}.b];

    FWP{i_g} = Polyhedron(A,b);
    FWP{i_g}.minHRep;
    FWP{i_g}.minVRep;
end

end

function FWP = get_FWP_oneLeg(J)

p = getParams;
umax = p.umax;
mu = p.mu;

A = [[1 0;...
     -1 0;...
      0 1;...
      0 -1]*J';...
      1 -mu;...
     -1 -mu];
     
b = [umax * [1;1;1;1];...
         0;...
         0];

FWP = Polyhedron(A,b);
FWP.minHRep;
FWP.minVRep;

end

function F = cst_Mccomick(x_in,M,x_interval,y_interval)
% x = linspace(x_interval(1),x_interval(2),21);
% y = linspace(y_interval(1),y_interval(2),21);
% [X,Y] = meshgrid(x, y);
% w = X.*Y;
% surf(X,Y,w);

%Piecewise McCormick Bilinear Estimation
x_min = x_interval(1);
x_max = x_interval(2);

y_min = y_interval(1);
y_max = y_interval(2);

bin = binvar(M(1),M(2));
x_grid = linspace(x_min,x_max,M(1)+1);
y_grid = linspace(y_min,y_max,M(2)+1);

H = zeros(6,3,M(1),M(2));
k = zeros(6,M(1),M(2));

bigM = 100;

F = [];
F = [F,sum(sum(bin)) == 1];

%% McCormick Envelope
for ii = 1:M(1)
    for jj = 1:M(2)
        xiL = x_grid(ii);
        xiU = x_grid(ii+1);
        yiL = y_grid(jj);
        yiU = y_grid(jj+1);
        H(1,1:3,ii,jj) = [yiL xiL -1];
        k(1,ii,jj) = yiL*xiL;
        H(2,1:3,ii,jj) = [yiU xiU -1];
        k(2,ii,jj) = yiU*xiU;
        H(3,1:3,ii,jj) = [-yiU -xiL 1];
        k(3,ii,jj) = -xiL*yiU;
        H(4,1:3,ii,jj) = [-yiL -xiU 1];
        k(4,ii,jj) = -yiL*xiU;
        H(5,1:3,ii,jj) = [1 0 0];
        k(5,ii,jj) = xiU;
        H(6,1:3,ii,jj) = [-1 0 0];
        k(6,ii,jj) = -xiL;
        
        F = [F, H(:,:,ii,jj)*x_in - k(:,ii,jj) - bigM * (1 - bin(ii,jj)) <= 0];
        
%         plot(H(:,:,ii,jj)*x_in <= k(:,ii,jj));hold on
    end
end

end

function F = cst_quadratic_approx(x,y,M,x_interval)
xmin = x_interval(1);
xmax = x_interval(2);
x_grid = linspace(xmin,xmax,M+1);

bigM = 100;
b = binvar(M,1);
F = [];
F = [F,xmin <= x <= xmax];
F = [F,sum(b) == 1];
for ii = 1:M
    xl = x_grid(ii);
    xu = x_grid(ii+1);
    yl = xl^2;
    yu = xu^2;
    
    F = [F,-bigM*(1-b(ii)) + xl <= x <= xu + bigM*(1-b(ii))];
    
    k = (yu - yl)/(xu - xl);
    F = [F,-bigM*(1-b(ii)) + k*(x-xl)+yl <= y <= k*(x-xl)+yl + bigM*(1-b(ii))];
end

end

function xval = get_value(x,Nstep)
    for istep = 1:Nstep
        xval(:,:,istep) = value(x{istep});

    end
end

function fig_plot2D(val)

p = getParams;
g = p.g;
M_st = p.M_st;
M_air = p.M_air;

t_vec = val.t;
q = val.q;
dq = val.dq;
qTD = val.qTD;
Tst = val.Tst;
Tair = val.Tair;
qend = val.qend;

Nstep = size(t_vec,1);

%%
q_st_list = cell(Nstep,1);
q_fl_list = cell(Nstep,1);

q_list = [];
t_list = [];

for istep = 1:Nstep

    t_st = linspace(0,Tst(istep),M_st);     % stance
    t_air = linspace(0,Tair(istep),M_air);  % aerial
    
    q_st_list{istep} = repmat(qTD(:,istep),[1,M_st]) + q(:,:,istep);
    q_TO = qTD(:,istep) + q(:,end,istep);   % take-off state
    q_fl_list{istep} = repmat(q_TO,[1,M_air]) + dq(:,end,istep)*t_air + [0;-1/2*g]*t_air.^2;
    
    if istep == 1
        t_list = [t_st(1:end-1),t_st(end) + t_air];
    else
        t_list = [t_list(end)+t_st(1:end-1),t_list(end)+t_st(end)+t_air];
    end

    q_list = [q_list,q_st_list{istep},q_fl_list{istep}];
    
    %%
    hold on;axis equal;grid on;box on
    % CoM trajectory in stance
    plot(q_st_list{istep}(1,:),q_st_list{istep}(2,:),'r','linewidth',1)
    % CoM traj in air
    plot(q_fl_list{istep}(1,:),q_fl_list{istep}(2,:),'b','linewidth',1)
    % qTD
    plot(qTD(1,istep),qTD(2,istep),'rx','markersize',12,'linewidth',2)

end

% qTD at the end
plot(qTD(1,end),qTD(2,end),'rx','markersize',12,'linewidth',2)

end

% Function to evaluate bezier polynomials
% Inputs: Alpha - Bezeir coefficients (alpha_0 ... alpha_M)
%         s - s parameter. Range [0 1]
% Outputs: b = sum(k=0 to m)[ alpha_k * M!/(k!(M-k)!) s^k (1-s)^(M-k)]
%factorial(M)/(factorial(k)*factorial(M-k))
function b = polyval_bz(alpha, s)
    b = zeros(size(s)) ;
    M = size(alpha,2)-1 ;  % length(alpha) = M+1
    switch M
        case 2
            c = [1 2 1];
        case 3
             c = [1 3 3 1];
        case 4
            c = [1 4 6 4 1];
        case 5
            c = [1 5 10 10 5 1];
        case 6
            c = [1 6 15 20 15 6 1];
        case 7
            c = [1 7 21 35 35 21 7 1];
        case 8
            c = [1 8 28 56 70 56 28 8 1];
        case 9
            c = [1 9 36 84 126 126 84 36 9 1];
        case 10
            c = [1 10 45 120 210 252 210 120 45 10 1];
        otherwise
            c = 0;
    end
        
    for k = 0:M
        
        b = b + alpha(:,k+1) .* factorial(M)/(factorial(k)*factorial(M-k)) .* s.^k .* (1-s).^(M-k) ;
    end
end








