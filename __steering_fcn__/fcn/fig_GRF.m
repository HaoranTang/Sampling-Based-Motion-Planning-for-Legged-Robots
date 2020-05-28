function fig_GRF(Fx_var,Fz_var,Tst,Nstep,p)

mass = p.mass;
g = p.g;

for istep = 1:Nstep
    
Fx_co = [0 Fx_var(istep,:) 0];
Fz_co = [mass*g Fz_var(istep,:) 0];

s = linspace(0,1,101);
Fx = polyval_bz(Fx_co,s);
Fz = polyval_bz(Fz_co,s);

figure
plot(s*Tst(istep),Fx)
hold on
plot(s*Tst(istep),Fz)
legend('Fx','Fz')
end






