function [C,Ceq] = myConstraints(z,pack,dynFun,pathCst,bndCst)

% --- unpack variables ---
[t,x,u] = unPackDecVar(z,pack);

dt = (t(end)-t(1))/(length(t)-1);

% --- dynamics function ---
f = dynFun(t,x,u);

% --- calculate defects ---
defects = defectCst(dt,x,f);

% --- collect the constraints ---
[C, Ceq] = collectConstraints(t,x,u,defects,pathCst,bndCst);

end

function defects = defectCst(dt,x,f)

nTime = size(x,2);

idxLow = 1:(nTime-1);
idxUpp = 2:nTime;

xLow = x(:,idxLow);
xUpp = x(:,idxUpp);

fLow = f(:,idxLow);
fUpp = f(:,idxUpp);

% This is the key line:  (Trapazoid Rule)
defects = xUpp-xLow - 0.5*dt*(fLow+fUpp);

end










