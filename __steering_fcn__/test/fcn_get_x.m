function px = fcn_get_x(co1,co2)

fcn_para = @(x,co)co(1)+co(2)*x+co(3)*x.^2;


x_sym1 = fcn_get_Asym(co1);
x_sym2 = fcn_get_Asym(co2);

x_range = [min(x_sym1,x_sym2),max(x_sym1,x_sym2)];

cox = co1 - co2;
c = cox(1);
b = cox(2);
a = cox(3);

xx1 = 1/(2*a) * (-b+sqrt(b^2-4*a*c));
xx2 = 1/(2*a) * (-b-sqrt(b^2-4*a*c));

if (x_range(1) < xx1) && (xx1 < x_range(2))
    x = xx1;
else
    x = xx2;
end

px = [x;fcn_para(x,co1)];

end

function x_sym = fcn_get_Asym(co)

c = co(1);
b = co(2);
a = co(3);

x_sym = -b/(2*a);

end

