function fig_plotGrid(grid0,style)

hold on
box on
grid on

N = length(grid0);
c_gray = [211 211 211]/255;
c_blue = [0 0.447 0.741];

for i_N = 1:N
    g = grid0{i_N};
%     plot(g(:,1),g(:,2),style)
    plot(g([1:3,1],1),g([1:3,1],2),'color',c_blue,'linewidth',1)
    
    J1 = getJacobian(g(1,:));
    J2 = getJacobian(g(2,:));
    J3 = getJacobian(g(3,:));
    
    condJ = 1/3 * (cond(J1) + cond(J2) + cond(J3));

end

axis equal;


% axis([-0.2 0.3 -0.02 0.28])
% hfig = figure(1);
% set(hfig,'Position',[100,300,500,300]);

% th = 12/180*pi;
% plot([-0.2 0.3],[0.2*tan(th) -0.3*tan(th)],'color','k','linewidth',2)
% plot(0,0,'ro','linewidth',2,'markersize',12)
% 
% xlabel('x [m]','fontsize',16)
% ylabel('z [m]','fontsize',16)
% set(gca,'fontsize',12)


end

function J = getJacobian(pt)
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

J = fcn_J([q1,q2],[l1,l2]);

end