function [Fx,Fz,dx,dz,x,z,Fx_co,Fz_co] = fcn_defVar(Fx_var,Fz_var,x0,z0,dx0,dz0,Tst,M_st,Nstep,p)

mass = p.mass;
g = p.g;

for istep = 1:Nstep

    Fx_co(istep,:) = [0 Fx_var(istep,:) 0];
    Fz_co(istep,:) = [mass*g Fz_var(istep,:) 0];
    dx_co(istep,:) = bz_int(Fx_co(istep,:)/mass,dx0(istep),Tst(istep));
    dz_co(istep,:) = bz_int(Fz_co(istep,:)/mass - g,dz0(istep),Tst(istep));
    x_co(istep,:) = bz_int(dx_co(istep,:),x0(istep),Tst(istep));
    z_co(istep,:) = bz_int(dz_co(istep,:),z0(istep),Tst(istep));

    t_vec = linspace(0,Tst(istep),M_st);
    s_vec = t_vec/Tst(istep);

    Fx(istep,:) = polyval_bz(Fx_co(istep,:),s_vec);
    Fz(istep,:) = polyval_bz(Fz_co(istep,:),s_vec);
    dx(istep,:) = polyval_bz(dx_co(istep,:),s_vec);
    dz(istep,:) = polyval_bz(dz_co(istep,:),s_vec);
    x(istep,:) = polyval_bz(x_co(istep,:),s_vec);
    z(istep,:) = polyval_bz(z_co(istep,:),s_vec);

end