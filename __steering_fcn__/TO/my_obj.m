function obj = my_obj(z,pack,p)

g = p.g;
vin = p.vin;
vout = p.vout;

[t,Xi,co,Q,tau] = unpack_dec_var(z,pack);

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

xi = x(1);
zi = z(1);
xf = x(end);
zf = z(end);

norm_posi = norm(zi + (g/(2*vin(1)^2))*xi^2 - (vin(2)/vin(1)) * xi);
norm_posf = norm(zf + (g/(2*vout(1)^2))*xf^2 - (vout(2)/vout(1)) * xf);

obj = norm_posi + norm_posf;

end