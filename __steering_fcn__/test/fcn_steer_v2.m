% single leg MICP motion planning
function [bool_success,val] = fcn_steer(vin,vout)
% bool_success

% ---- parameters -----
p = getParams();
Nstep = 1;          % number of steps

mass = p.mass;
g = p.g;
Nr = p.Nr;          % number of nodes in r direction
Nth = p.Nth;        % number of nodes in th direction
M_st = p.M_st;      % number of sampling points in stance phase
Tst = 0.15*ones(Nstep,1);   % stance time
bigM = 100;
xmin = -0.2;
xmax = 0.2;

%% Grid and FWP
load('grid_FWP.mat')
fprintf('Grid and FWP loaded!\n')

%% Define variables
Ngrid = length(Grid);

% binary variables
b_WS = binvar(M_st,Ngrid,Nstep);    % binvar for workspace

% % boundary condition
% Xi = sdpvar(4,Nstep,'full');
% Xf = sdpvar(4,Nstep,'full');
sdpvar xi zi dxi dzi

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
    
    % B-coefficient
    Fx_co(istep,:) = [0 Fx_var(istep,:) 0];
    Fz_co(istep,:) = [0 Fz_var(istep,:) 0];
    
    ddx_co(istep,:) = Fx_co(istep,:) / mass;
    ddz_co(istep,:) = Fz_co(istep,:) / mass - g;
    dx_co(istep,:) = bz_int(ddx_co(istep,:),dxi,Tst(istep));
    dz_co(istep,:) = bz_int(ddz_co(istep,:),dzi,Tst(istep));
    x_co(istep,:) = bz_int(dx_co(istep,:),xi,Tst(istep));
    z_co(istep,:) = bz_int(dz_co(istep,:),zi,Tst(istep));
    
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

fprintf('variable definition finished!\n')

%% Constraints
F = [];
for istep = 1:Nstep
    % temporal uniqueness
    F = [F;(sum(b_WS(:,:,istep),2) == 1):'sum_b_WS'];

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
    % Xin and Xout stay on the parabola
    Xf = [q{istep}(:,end);dq{istep}(:,end)];
    sdpvar xf zf xf2 xi2
    F = [F;(xf == Xf(1)):'xf == Xf(1)'];
    F = [F;(zf == Xf(2)):'zf == Xf(2)'];
    dxf = Xf(3);
    dzf = Xf(4);
    F = [F;(dxi == vin(1)):'dxi = vin(1)'];
    F = [F;(dxf == vout(1)):'dxf'];
    F = [F;(dzi == vin(2)):'dzi(x)'];
    F = [F;(dzf == vout(2)):'dzf(x)'];
end

sdpvar si sf

Fquad = cst_quadratic_approx(xf,xf2,12,[xmin;xmax]);
F = [F;Fquad:'xf^2 = xf2'];
% sf = zf + (g/(2*vout(1)^2))*xf2 - (vout(2)/vout(1)) * xf;

Fquad = cst_quadratic_approx(xi,xi2,12,[xmin;xmax]);
F = [F;Fquad:'xi^2 = xi2'];
% si = zi + (g/(2*vin(1)^2))*xi2 - (vin(2)/vin(1)) * xi;

fprintf('constraint definition finished!\n')

%% Optimize!
% obj = -qTD(1,end);
normi = norm(zf-0.1 + (g/(2*vout(1)^2))*xf2 - (vout(2)/vout(1)) * xf);
normf = norm(zi-0.1 + (g/(2*vin(1)^2))*xi2 - (vin(2)/vin(1)) * xi);

obj = normi + normf;
ops = sdpsettings('solver','gurobi','debug',1,'verbose',1);
sol = optimize(F,obj,ops);

if sol.problem == 0
    bool_success = 1;
    val.q = get_value(q,Nstep);
    val.dq = get_value(dq,Nstep);
    val.residual = value(obj);
    val.Fx_var = get_value(Fx_var,Nstep);
    val.Fz_var = get_value(Fz_var,Nstep);
    
    fig_plot2D(val)
else
    bool_success = 0;
end

end

function p = getParams()

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
p.M_st = 10;                   % number of sampling points in stance phase
p.M_air = 21;                 % number of sampling points in aerial phase

p.c_red = [0.85 0.325 0.098];
p.c_blue = [0 0.447 0.741];
p.c_orange = [255 179 0]/255;

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
    
    bigM_binary = bigM*(1-b(ii));
    
    F = [F,-bigM_binary + xl <= x <= xu + bigM_binary];
    
    k = (yu - yl)/(xu - xl);
    F = [F,-bigM_binary <= y - (k*(x-xl)+yl) <= bigM_binary];
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

function xval = get_value(x,Nstep)
    for istep = 1:Nstep
        xval(:,:,istep) = value(x{istep});
    end
end

function fig_plot2D(val)

scale = 1e-1;

q = val.q;
dq = val.dq;

hold on;axis equal;grid on;box on
len = length(q);
% touchdown point
plot(0,0,'rx','markersize',12,'linewidth',2)

% CoM trajectory in stance
plot(q(1,:),q(2,:),'b','linewidth',1)
for ii = 1:len
    chain = [q(:,ii),q(:,ii)+dq(:,ii)*scale];
    plot(chain(1,:),chain(2,:),'r')    
end

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








