%% geometry analysis for ballistic motion planning
g = 9.81;

xs = [0;0];
xg = [1;0.4];
xsg = xg - xs;
hold on
plot(xs(1),xs(2),'rx')
plot(xg(1),xg(2),'rx')

%% find vs to shoot to the goal
th = atan2(xsg(2),xsg(1));
alpha = pi*0.35;
if alpha < th
    alpha = th;
end

dx = sqrt((g*xsg(1))/(2*(xsg(1)*tan(alpha)-xsg(2))));
dz = dx * tan(alpha);

vs = [dx;dz];
% vs_norm = norm(vs)
Tair = xsg(1) / dx;
t = linspace(0,Tair,101);

xt = repmat(xs,size(t)) + vs * t + 1/2*[0;-g]*t.^2;

plot(xt(1,:),xt(2,:),'r','linewidth',2)
axis equal; hold on;


%% parametrize parabola w.r.t. x
plot(xt(1,:),xt(2,:))
axis equal
x = linspace(0,1,101);
% z(x)
z = -g/(2*dx^2) * x.^2 + dz/dx * x;
plot(x,z,'b--','linewidth',2)

% vx(x) = dx; vz(x)
x_ = linspace(0,1,11);
z_ = -g/(2*dx^2) * x_.^2 + dz/dx * x_;
vx = dx * ones(size(x_));
vz = dz * ones(size(x_)) - g/dx * x_;
for ii = 1:length(x_)
    p1 = [x_(ii);z_(ii)];
    p2 = p1 + [vx(ii);vz(ii)]*0.2;
    chain = [p1,p2];
    plot(chain(1,:),chain(2,:),'g','linewidth',1)
end


%% optimize on parabola
x1 = sdpvar(2,1);
x2 = sdpvar(2,1);
sdpvar x_2

F = [];
F = [F;x1(2) == -x1(1)^2];
F = [F;x2(2) == -x2(1) + 2];
% Fquad = cst_quadratic_approx(x1(1),x_2,20,[-1;1]);
% F = [F;Fquad];

ops = sdpsettings('solver','fmincon','debug',1,'verbose',1);
obj = norm(x1 - x2);
sol = optimize(F,obj,ops)



%%
% z = c0 + c1 * x + c2 * x^2
% vx = sqrt(-g/(2*c2))
% vz = (c1 + 2*c2*x)*vx
% 





