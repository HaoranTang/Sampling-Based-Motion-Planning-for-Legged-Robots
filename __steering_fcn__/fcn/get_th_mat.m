function th_mat = get_th_mat(rSpan,Nth,th_Max,thMin)

Nr = length(rSpan);
th_mat = zeros(Nr,Nth);

    for ii = 1:Nr
        thMax = min(th_Max,find_th_max(rSpan(ii)));
        th_mat(ii,:) = linspace(thMax,thMin,Nth);
    end

end


function th = find_th_max(r)

l = 0.15;
slope = 0;

syms x y
eqns = [(y + l*sin(slope))^2 + (x + l*cos(slope))^2 == l^2, x^2 + y^2 == r^2];
vars = [x,y];
[solx,soly] = solve(eqns,vars);


xx = solx(1);
yy = vpa(soly(2));

th = atan2(-xx,yy) + pi/2;
th = vpa(th,5);

end