function [C,Ceq] = my_cst(z,pack,p)

nGrid = p.nGrid;
g = p.g;
vin = p.vin;
vout = p.vout;

C = [];
Ceq = [];

% unpack decision variables
[t,Xi,co,Q,tau] = unpack_dec_var(z,pack);
q = Q(1:2,:);
dq = Q(3:4,:);

x0 = Xi(1);
z0 = Xi(2);
dx0 = Xi(3);
dz0 = Xi(4);

s = t / t(end);

coFx = [0 co(1,:) 0];
coFz = [0 co(2,:) 0];
covx = bz_int(coFx,dx0,t(end));
covz = bz_int(coFz,dz0,t(end));
cox = bz_int(covx,x0,t(end));
coz = bz_int(covz,z0,t(end));

Fx = polyval_bz(coFx,s);
Fz = polyval_bz(coFz,s);
vx = polyval_bz(covx,s);
vz = polyval_bz(covz,s);
x = polyval_bz(cox,s);
z = polyval_bz(coz,s);

% boundary condition
xi = x(1);
zi = z(1);
xf = x(end);
zf = z(end);
vxi = vx(1);
vzi = vz(1);
vxf = vx(end);
vzf = vz(end);
Ceq(end+1) = vxi - vin(1);
Ceq(end+1) = vxf - vout(1);
Ceq(end+1) = vzi - (vin(2) - (g/vin(1)) * xi);
Ceq(end+1) = vzf - (vout(2) - (g/vout(1)) * xf);

for ii = 1:nGrid
    % time constraint
    Ceq(end+1) = t(end) - p.Tst;
    
    % kinematics constraint
    q1 = q(1,ii);
    q2 = q(2,ii);
    q12 = q1 + q2;
    J = fcn_J(q(:,ii),[p.l1,p.l2]);
    Ceq(end+1) = x(ii) + p.l1 * cos(q1) + p.l2 * cos(q12);
    Ceq(end+1) = z(ii) + p.l1 * sin(q1) + p.l2 * sin(q12);

    Ceq(end+1:end+2) = [vx(ii);vz(ii)] + J * dq(:,ii);
    
    C(end+1) = norm([x(ii);z(ii)]) - p.maxLegExt;

    % torque constraint
    Ceq(end+1:end+2) = tau(:,ii) + J' * [Fx(ii);Fz(ii)];
    Ceq(end+1:end+2) = tau(:,ii) - p.taumax;
    Ceq(end+1:end+2) = -tau(:,ii) - p.taumax;

    % GRF constraint
    C(end+1) = Fx(ii) - p.mu * Fz(ii);
    C(end+1) = -Fx(ii) - p.mu * Fz(ii);
    C(end+1) = -Fz(ii);
end





end







