%% test terrain
close all
clear all
yalmip('clear')

% plot obstacle
hold on;
fcn_plot_obstacle([40, 0, 80, 20])
fcn_plot_obstacle([105, 0, 120, 50])
axis([0.4 1.2 0 1])
xlabel('x [m]','fontsize',16)
ylabel('z [m]','fontsize',16)

wayPts = [[90;0.1],[76;20.1],[110;50.1]]/100;
% coef = [vel th time dir(1-right,2-left)]
coef = [[211.77679704042453, 1.317122230656279, 0.26341555592263943, 2]',...
         [276.5885934251446, 1.223734722638664, 0.36140316127590094, 1]'];
     
% P - pos, vin, vout
P = fcn_preprocess(wayPts,coef);

% plot trajectory
plot_traj(P)

% first jump
[bool_success,val1] = fcn_steer_init(P{1}.vout);
fig_plot2D(val1,wayPts(:,1));

% second jump
[bool_success,val2] = fcn_steer(P{2}.vin,P{2}.vout);
fig_plot2D(val2,wayPts(:,2));

%% Terrain: rocky road
close all
yalmip('clear')

obstacle = [[40, 0, 60, 30];...
            [90, 0, 120, 20];...
            [150, 0, 180, 40];...
            [0, -50, 200, 0]];
plot_obstacle(obstacle)
xlim([0 2])


wayPts = ...
[[0; 0.1],...
 [17.67168203976844; 0.1],...
 [53.64199222356509; 30.1],...
 [112.59448148435428; 20.1],...
 [155.1702435799325; 40.1],...
 [196.41703665302464; 0.1]]/100;
% coef = [vel th time dir(1-right,2-left)]
coef = ...
[[200.0, 1.3469114639366924, 0.39797644698577844, 1]',...
 [279.3215971478916, 1.2121951819050851, 0.3669240876917547, 1]',...
 [222.24920238964285, 0.7853981633974483, 0.3751258000179715, 1]',...
 [286.6343714379993, 1.2473068943547025, 0.46727780272761504, 1]',...
 [277.533244501892, 1.3483389525467628, 0.6736220656245537, 1]'];

% P - pos, vin, vout
P = fcn_preprocess(wayPts,coef);
% plot trajectory
plot_traj(P)

PATH = {};
% first jump
[bool_success,val] = fcn_steer_init(P{1}.vout);
fig_plot2D(val,wayPts(:,1));
PATH{1} = val;

for ii = 2:size(wayPts,2)-1
    [bool_success,val] = fcn_steer(P{ii}.vin,P{ii}.vout);
    fig_plot2D(val,wayPts(:,ii));
    PATH{ii} = val;
end

info_simple.obstacle = obstacle;
info_simple.P = P;
info_simple.PATH = PATH;
info_simple.wayPts = wayPts;
info_simple.start = [0;0];
info_simple.goal = [2;0];
info_simple.axis = [0 2 -0.3 1];

animate_jump(info_simple)

T_tot = 0;
for ii = 1:length(PATH)
    T_tot = T_tot + PATH{ii}.Tsolve;
    fprintf('jump %d solve time is %.2f s\n',ii,PATH{ii}.Tsolve)
end
fprintf('Total solve time is %.2f s\n',T_tot)

%% --------------- old path -----------------
obstacle = [[40, 0, 60, 30];...
            [90, 0, 120, 20];...
            [150, 0, 180, 40];...
            [0, -50, 200, 0]];
plot_obstacle(obstacle)

wayPts_old =...
[[0, 0.1]',...
 [17.67168203976844, 0.1]',...
 [53.64199222356509, 30.1]',...
 [91.81090155208663, 20.1]',...
 [112.59448148435428, 20.1]',...
 [129.02407541717042, 0.1]',...
 [155.1702435799325, 40.1]',...
 [196.41703665302464, 0.1]']/100;
coef_old = ...
[[200.0, 1.3469114639366924, 0.39797644698577844, 1]',...
 [279.3215971478916, 1.2121951819050851, 0.3669240876917547, 1]',...
 [200.0, 1.088118256757697, 0.4111673665288606, 1]',...
 [200.00000000000003, 1.3036701569141724, 0.3936871250230664, 1]',...
 [200.0, 1.4010364704107543, 0.4862389986266364, 1]',...
 [297.537092907853, 1.3292820133955134, 0.3674128194881509, 1]',...
 [277.533244501892, 1.3483389525467628, 0.6736220656245537, 1]'];

% P - pos, vin, vout
P = fcn_preprocess(wayPts_old,coef_old);
% plot trajectory
plot_traj(P)
axis(info_simple.axis)

%% Terrain: Jackie Chan
close all
yalmip('clear')

% plot obstacle
obstacle = [[40, 0, 60, 100];...
            [60,0,75,46];...
            [75, 0, 88, 20];...
            [100, 0, 120, 75];...
            [120, 0, 130, 100]];
plot_obstacle(obstacle)

% path
wayPts = [[93; 0.1],...
 [83.85492750059072; 20.1],...
 [73.50879812041435; 46.1],...
 [101.87075306321431; 75.1]]/100;

coef = [[250.99945829502485, 1.4820071267976518, 0.4108894628112192, 2]',...
 [258.6454607922643, 1.467634660910602, 0.38844120111066927, 2]',...
 [280.3813697912152, 1.3216886048100647, 0.4102993348726899, 1]'];

% P - pos, vin, vout
P = fcn_preprocess(wayPts,coef);
% plot trajectory
plot_traj(P)

PATH = {};
% first jump
[bool_success,val] = fcn_steer_init(P{1}.vout);
fig_plot2D(val,wayPts(:,1));
PATH{1} = val;

for ii = 2:size(wayPts,2)-1
    [bool_success,val] = fcn_steer(P{ii}.vin,P{ii}.vout);
    fig_plot2D(val,wayPts(:,ii));
    PATH{ii} = val;
end

info_JC.obstacle = obstacle;
info_JC.P = P;
info_JC.PATH = PATH;
info_JC.wayPts = wayPts;
info_JC.start = [0.9;0];
info_JC.goal = [1.02;0.75];
info_JC.axis = [0.6 1.1 0 1];

animate_jump(info_JC)

T_tot = 0;
for ii = 1:length(PATH)
    T_tot = T_tot + PATH{ii}.Tsolve;
    fprintf('jump %d solve time is %.2f s\n',ii,PATH{ii}.Tsolve)
end
fprintf('Total solve time is %.2f s\n',T_tot)

%% Terrain: Indiana Jones
close all
yalmip('clear')

% obstacle
obstacle = [[40, 0, 60, 30];...
            [86,30,100,100];...
            [135, 0, 155, 30];...
            [0, -50, 200 0]];
plot_obstacle(obstacle)

% path info
wayPts = ...
[[0; 0.1],...
 [47.36487660583829; 30.1],...
 [74.16186572051518; 0.1],...
 [114.25608237316908; 0.1],...
 [141.02355076936982; 30.1],...
 [198.37094205011223; 0.1]]/100;
% coef = [vel th time dir(1-right,2-left)]
coef = ...
[[295.3271090858333, 1.1553196737243876, 0.3973506049790691, 1]',...
 [200.0, 1.3067059585821863, 0.5132907631118189, 1]',...
 [200.00000000000003, 0.8795897381816608, 0.3144805776554967, 1]',...
 [266.7755362374796, 1.2715628357709283, 0.3403703517034835, 1]',...
 [200.00000000000003, 0.8813388786561026, 0.45076018049096195, 1]'];
% P - pos, vin, vout
P = fcn_preprocess(wayPts,coef);
% plot trajectory
plot_traj(P)

PATH = {};
% first jump
[bool_success,val] = fcn_steer_init(P{1}.vout);
fig_plot2D(val,wayPts(:,1));
PATH{1} = val;

for ii = 2:size(wayPts,2)-1
    [bool_success,val] = fcn_steer(P{ii}.vin,P{ii}.vout);
    fig_plot2D(val,wayPts(:,ii));
    PATH{ii} = val;
end

info_hard.obstacle = obstacle;
info_hard.P = P;
info_hard.PATH = PATH;
info_hard.wayPts = wayPts;
info_hard.start = [0;0];
info_hard.goal = [2;0];
info_hard.axis = [0 2 -0.5 0.8];

animate_jump(info_hard)

T_tot = 0;
for ii = 1:length(PATH)
    T_tot = T_tot + PATH{ii}.Tsolve;
    fprintf('jump %d solve time is %.2f s\n',ii,PATH{ii}.Tsolve)
end
fprintf('Total solve time is %.2f s\n',T_tot)


%%
% g = 9.81;
% mu = 1;
% p1 = [0;0];
% p2 = [0.4;0.3];
% sdpvar v th T real
% 
% F = [];
% F = [F;p1 + v*[cos(th);sin(th)]*T + 1/2*[0;-g]*T^2 == p2];
% F = [F;0 <= th <= pi/2];
% F = [F;5 >= v >= 0];
% F = [F;0.5>= T >= 0];
% obj = [];
% ops = sdpsettings('solver','fmincon');
% sol = optimize(F,obj,ops);
% 
% val.v = value(v);
% val.th = value(th);
% val.T = value(T);


% %%
% g = 9.81;
% mu = 1;
% p1 = [0;0];
% p2 = [0.4;0.3];
% sdpvar a b c real
% 
% F = [];
% F = [F;a*p1(1)^2+b*p1(1)+c == p1(2)];
% F = [F;a*p2(1)^2+b*p2(1)+c == p2(2)];
% F = [F;2*a*p1(1)+b >= pi/4];
% F = [F;2*a*p2(1)+b <= -pi/4];
% vx1 = sqrt(-g/(2*a));
% vz1 = vx1 * (2*a*p1(1)+b);
% 
% sol = optimize(F)
% 
% val.a = value(a);
% val.b = value(b);
% val.c = value(c);
% 
% 
% %%
% % given two parabolas, use fcn_steer to stich them together
% % assume qTD=(0,0), (xin,zin,dxin,dzin)-->(xout,zout,dxout,dzout)
% 
% vin = [0.6544;-1.0898];
% vout = [0.4552;2.5608];
% 
% [bool_success,val] = fcn_steer(vin,vout);






