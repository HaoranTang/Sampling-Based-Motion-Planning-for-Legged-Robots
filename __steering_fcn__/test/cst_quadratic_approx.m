function F = cst_quadratic_approx(x,y,M,x_interval)
xmin = x_interval(1);
xmax = x_interval(2);
x_grid = linspace(xmin,xmax,M+1);

bigM = 100;
b = binvar(M,1);
F = [];
F = [F,xmin <= x <= xmax];
F = [F,sum(b) == 1];
for ii = 1:M
    xl = x_grid(ii);
    xu = x_grid(ii+1);
    yl = xl^2;
    yu = xu^2;
    
    bigM_binary = bigM*(1-b(ii));
    
    F = [F,-bigM_binary + xl <= x <= xu + bigM_binary];
    
    k = (yu - yl)/(xu - xl);
    F = [F,-bigM_binary <= y - (k*(x-xl)+yl) <= bigM_binary];
end

end