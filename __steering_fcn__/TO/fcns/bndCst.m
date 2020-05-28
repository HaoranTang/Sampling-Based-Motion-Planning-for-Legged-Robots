function [c,ceq] = bndCst(x0,xf,p)

[dxf,dyf,dthf] = deal(xf(4),xf(5),xf(6));

c = [];
ceq = [];

c(end+1) = dxf - 1;
c(end+1) = -dxf - 1;
c(end+1) = dyf - 1;
c(end+1) = -dyf - 1.5;
c(end+1) = dthf - 6;
c(end+1) = -dthf - 6;

c = reshape(c,numel(c),1);
ceq = reshape(ceq,numel(ceq),1);

end