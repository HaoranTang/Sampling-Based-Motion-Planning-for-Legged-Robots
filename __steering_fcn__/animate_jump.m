function animate_jump(info)

set(gcf, 'Color', 'white')

name = ['test.mp4'];
vidfile = VideoWriter(name,'MPEG-4');
open(vidfile);

tic
clf; hold on;


% plot obstacle
plot_obstacle(info.obstacle)

% plot trajectory
plot_traj(info.P)
axis(info.axis)
grid off;box on
% plot goal
plot(info.goal(1),info.goal(2),'go','markersize',15,'linewidth',1)


while toc < 0.5
    writeVideo(vidfile, getframe(gcf));
end
tic

% PATH
PATH = info.PATH;

for ii = 1:length(PATH)
    clf; hold on;
    % plot obstacle
    plot_obstacle(info.obstacle)

    % plot trajectory
    plot_traj(info.P)
    fig_plot2D(PATH{ii},info.wayPts(:,ii));
    
    % plot goal
    plot(info.goal(1),info.goal(2),'go','markersize',15,'linewidth',1)
    
    axis(info.axis)
    grid off
    
    while toc < 0.5
        writeVideo(vidfile, getframe(gcf));
    end
    tic
end

for ii = 1:length(PATH)
    fig_plot2D(PATH{ii},info.wayPts(:,ii));
end
axis(info.axis)
grid off

while toc < 0.5
    writeVideo(vidfile, getframe(gcf));
end


close(vidfile);
end