function [t,Xi,co,Q,U] = unpack_dec_var(z,pack)

nt = pack.nt;
nXi = pack.nX;
nco = pack.nco;
nX = pack.nX;
nU = pack.nU;

t = linspace(z(1),z(2),nt);
Xi = z(pack.xiIdx);
co = z(pack.coIdx);
Q = z(pack.xIdx);
U = z(pack.uIdx);


end