function plot_robot(pos,p)
l1 = p.l1;
l2 = p.l2;

q = IK(pos,p);

q1 = q(1);
q12 = sum(q);
p_knee = pos + l1 * [cos(q1);sin(q1)];
p_foot = p_knee + l2 * [cos(q12);sin(q12)];

chain = [pos,p_knee,p_foot];
hold on
plot(pos(1),pos(2),'ro','markersize',20,'linewidth',2)
plot(chain(1,:),chain(2,:),'k','linewidth',2)
plot([-0.5 0.5],[0 0],'k')

axis equal
axis([-0.5 0.5 -0.1 0.5])



end





