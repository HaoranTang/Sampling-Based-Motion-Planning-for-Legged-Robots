% --- parameters
g = 9.81;

co1 = [-0.09888112980655206, 1.6607073404071746, 9.999999999999996];
co2 = [-0.11452998500361418, 5.512393287442574, -50.27491967498786];
co3 = [-0.2366801341544749, 20.458278790269087, -398.67141738395446];

%% --- fcn def ---
% the parabolic curve
fcn_para = @(x,co)co(1)+co(2)*x+co(3)*x.^2;

% change the unit of coefficients from cm to m
fcn_cm2m = @(co)diag([100,1,0.01])*co(:);


%%
% z = co(1) + co(2)*x + co(3)*x^2
co(:,1) = flipud(fcn_cm2m(co1));
co(:,2) = flipud(fcn_cm2m(co2));
co(:,3) = flipud(fcn_cm2m(co3));

%%
P{1}.co_in = [];
P{1}.co_out = co(:,1);
P{1}.pos = [0;co(1,1)];
P{1}.vin = [0;0];
P{1}.vout = fcn_get_vel(P{1}.pos(1),P{1}.co_out,1);

P{2}.co_in = co(:,1);
P{2}.co_out = co(:,2);
P{2}.pos = fcn_get_x(P{2}.co_in,P{2}.co_out);
P{2}.vin = fcn_get_vel(P{2}.pos(1),P{2}.co_in,1);
P{2}.vout = fcn_get_vel(P{2}.pos(1),P{2}.co_out,1);

P{3}.co_in = co(:,2);
P{3}.co_out = co(:,3);
P{3}.pos = fcn_get_x(P{3}.co_in,P{3}.co_out);
P{3}.vin = fcn_get_vel(P{3}.pos(1),P{3}.co_in,1);
P{3}.vout = fcn_get_vel(P{3}.pos(1),P{3}.co_out,1);

P{4}.co_in = co(:,3);
P{4}.co_out = [0;0;0];
P{4}.pos = fcn_get_x(P{4}.co_in,P{4}.co_out);
P{4}.vin = fcn_get_vel(P{4}.pos(1),P{4}.co_in,1);
P{4}.vout = [0;0];


%%
plot_para(co,P)












