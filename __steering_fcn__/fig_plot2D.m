function fig_plot2D(val,pos)

scale = 1e-1;

q = val.q;
dq = val.dq;

hold on;axis equal;grid on;box on
len = length(q);
% touchdown point
plot(pos(1),pos(2),'yx','markersize',8,'linewidth',1)

% CoM trajectory in stance
plot(pos(1)+q(1,:),pos(2)+q(2,:),'b','linewidth',1)
for ii = 1:len
    chain(1,:) = pos(1) + q(1,ii)+[0,dq(1,ii)*scale];
    chain(2,:) = pos(2) + q(2,ii)+[0,dq(2,ii)*scale];
    plot(chain(1,:),chain(2,:),'m')    
end

end