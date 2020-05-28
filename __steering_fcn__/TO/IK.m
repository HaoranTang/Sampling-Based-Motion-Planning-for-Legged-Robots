function q = IK(pos,p)

l1 = p.l1;
l2 = p.l2;

r = norm(pos);
th1 = atan2(pos(2),pos(1));
th2 = acos((r^2+l1^2-l2^2)/(2*r*l1));
q1 = -th1 - th2;

th3 = acos((l1^2+l2^2-r^2)/(2*l1*l2));
q2 = pi - th3;

q = [q1;q2];


end