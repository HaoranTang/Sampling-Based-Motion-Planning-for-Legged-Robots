
% get the knee position given hip and foot position
function pk = fcn_get_pk(ph,pf,p)

l = 0.14;
v_f2h = ph - pf;
th = atan2(v_f2h(2),v_f2h(1));
r = norm(v_f2h);
cos_alpha = (l^2+r^2-l^2)/(2*l*r);
alpha = acos(cos_alpha);

q1 = -(pi-th) - alpha;

pk = ph + l*[cos(q1);sin(q1)];


end