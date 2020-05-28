function p = getParams()

L = 0.3;
p.L = L;
p.l1 = 0.14;
p.l2 = 0.14;
p.mass = 1;
p.g = 9.81;
p.mu = 1;
p.umax = 10;
p.pf_bck = [-L/2;0];
p.pf_frt = [L/2;0];

p.eMargin = 0.1;        % margin to the edge of obstacle

p.scale = 5e-4;