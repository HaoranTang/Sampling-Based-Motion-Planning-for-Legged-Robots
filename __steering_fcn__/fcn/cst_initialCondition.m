function F = cst_initialCondition(istep,x0,z0,dx0,dz0,xTD,zTD,dx,dz,Tair,p)

g = p.g;

F = [];
if istep == 1
    F = [F,x0(istep) == 0];
    F = [F,z0(istep) == 0.1];
    F = [F,dx0(istep) == 0];
    F = [F,dz0(istep) == 0];
    F = [F,xTD(istep) == 0];
    F = [F,zTD(istep) == 0];
else
    F = [F,-0.02 <= x0(istep) <= 0.05];
    F = [F, 0.05 <= z0(istep) <= 0.17];
    F = [F,dx0(istep) == dx(istep-1,end)];
    F = [F,dz0(istep) == dz(istep-1,end) - g * Tair(istep-1)];
end




