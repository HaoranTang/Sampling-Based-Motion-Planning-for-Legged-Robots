function fig_plotRobot(chain)

p_fL = chain(:,1);
p_kneeL = chain(:,2);
p_hipL = chain(:,3);
p_hipR = chain(:,4);
p_kneeR = chain(:,5);
p_fR = chain(:,6);

hold on;grid on;box on;
axis equal
% body
chain_body = [p_hipL,p_hipR];
plot(chain_body(1,:),chain_body(2,:),'color','k','linewidth',5)

% hip
fig_plotCircle(p_hipL,0.03)
fig_plotCircle(p_hipR,0.03)

% leg
chain_legL = [p_hipL,p_kneeL,p_fL];
chain_legR = [p_hipR,p_kneeR,p_fR];

plot(chain_legL(1,:),chain_legL(2,:),'color','k','linewidth',2)
plot(chain_legR(1,:),chain_legR(2,:),'color','k','linewidth',2)


