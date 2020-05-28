function [c,ceq] = pathCst(t,x,u,p)

mu = p.mu;
L = p.L;

pf1 = [-L/2;0];
pf2 = [L/2;0];
maxLegExt = p.maxLegExt;

c = [];
ceq = [];

len = length(t);
for ii = 1:len
    pc = x(1:2,ii);
    th = x(3,ii);
    u1x = u(1,ii);
    u1y = u(2,ii);
    u2x = u(3,ii);
    u2y = u(4,ii);
    
    %% --- Constraints ---
    %%%% leg stance mode constriant
    if t(ii) > p.Tds
        ceq(end+1) = u2x;
        ceq(end+1) = u2y;
    else
        %%%% leg extension constraints
        ph2 = pc + L/2*[cos(th);sin(th)];
        c(end+1) = norm(pf2-ph2) - maxLegExt;
        c(end+1) = -u2y;
        c(end+1) = u2x - mu*u2y;
        c(end+1) = -u2x - mu*u2y;
    end

    %%%% GRF constraint
    c(end+1) = -u1y;
    c(end+1) = u1x - mu*u1y;
    c(end+1) = -u1x - mu*u1y;

    %%%% leg extension constraints
    ph1 = pc - L/2*[cos(th);sin(th)];
    c(end+1) = norm(pf1-ph1) - maxLegExt;

end



end