function plot_obstacle(obs)

hold on;
for ii = 1:size(obs,1)
    fcn_plot_obstacle(obs(ii,:))
end
% axis([0.6 1.1 0 1])
xlabel('x [m]','fontsize',16)
ylabel('z [m]','fontsize',16)

end

function fcn_plot_obstacle(in)

rectangle('Position',[in(1:2), in(3:4)-in(1:2)]/100,...
          'FaceColor',[0.8 0.8 0.8])

axis equal

end