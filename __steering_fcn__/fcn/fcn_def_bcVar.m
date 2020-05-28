function [x0,z0,dx0,dz0,xTD,zTD,...
          Tair,Tair2,dx_end,dz_end,dxTair,dzTair] = fcn_def_bcVar(Nstep)

dx0 = sdpvar(Nstep+1,1);    dz0 = sdpvar(Nstep+1,1);
x0 = sdpvar(Nstep,1);       z0 = sdpvar(Nstep,1);

xTD = sdpvar(Nstep+1,1);    zTD = sdpvar(Nstep+1,1);

Tair = sdpvar(Nstep,1);
Tair2 = sdpvar(Nstep,1);

dx_end = sdpvar(Nstep,1);   dz_end = sdpvar(Nstep,1);
dxTair = sdpvar(Nstep,1);   dzTair = sdpvar(Nstep,1);

