function fig_plotRobot_oneLeg(chain)

p_f = chain(:,1);
p_knee = chain(:,2);
p_hip = chain(:,3);


% body
fig_plotCircle(p_hip,0.03);

% leg
chain_leg = [p_hip,p_knee,p_f];
plot(chain_leg(1,:),chain_leg(2,:),'color','k','linewidth',2)


