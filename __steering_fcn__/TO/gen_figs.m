function gen_figs(z,pack,p)

[t,X,U] = unPackDecVar(z,pack);

%% GRF
figure(2)
set(gcf, 'Color', 'white')
clf;hold on

h_u1x = plot([0],[0],'r','linewidth',3);
h_u1y = plot([0],[0],'b','linewidth',3);
h_u2x = plot([0],[0],'r--','linewidth',3);
h_u2y = plot([0],[0],'b--','linewidth',3);

h_u1x.XData = t;
h_u1x.YData = U(1,:);
h_u1y.XData = t;
h_u1y.YData = U(2,:);

h_u2x.XData = t;
h_u2x.YData = U(3,:);
h_u2y.XData = t;
h_u2y.YData = U(4,:);

legend('u1x','u1y','u2x','u2y')
xlabel('Time [s]')
ylabel('Force [N]')

drawnow

%% States
figure(3)
set(gcf, 'Color', 'white')
clf;hold on

% --- pos ---
subplot(2,1,1)
hold on
yyaxis left
plot(t,X(1:2,:),'linewidth',3)
ylabel('Position [m]')

yyaxis right
plot(t,X(3,:),'linewidth',3)
ylabel('Angle [m]')

legend('x','y','th','location','north')
xlabel('Time [s]')

% --- vel ---
subplot(2,1,2)
hold on
yyaxis left
plot(t,X(4:5,:),'linewidth',3)
ylabel('Velocity [m/s]')

yyaxis right
plot(t,X(6,:),'linewidth',3)
ylabel('Angular Velocity [rad/s]')

legend('dx','dy','dth','location','south')
xlabel('Time [s]')

end