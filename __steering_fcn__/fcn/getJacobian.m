function [J,q] = getJacobian(pt)
p = getParams;
l1 = p.l1;
l2 = p.l2;

th = atan2(pt(2),pt(1));
r = norm(pt,2);
cos_alpha = (l1^2+r^2-l2^2)/(2*l1*r);
alpha = acos(cos_alpha);

cos_beta = (l1^2+l2^2-r^2)/(2*l1*l2);
beta = acos(cos_beta);

q1 = -(pi - th + alpha);
q2 = pi - beta;
q = [q1;q2];

J = fcn_J([q1,q2],[l1,l2]);

end