% Trajectory Optimization for single leg stance
% Authors: Yanran Ding, Matthew Kelly
% Last Modified: 2020/05/06

% clear;clc;
addpath fcns
p.flag_movie = 1;               % make movie

for iter = 1
    tic
    
%% Parameters
if iter == 1
    nGrid = 10;
elseif iter == 2
    nGrid = 27;
end

p.L = 0.3;
p.nGrid = nGrid;
p.m = 2;
p.g = 9.81;
p.nX = 4;
p.nU = 2;
p.mu = 0.6;
p.maxLegExt = 0.22;
p.Tst = 0.15;

duration = p.Tst;
maxForce = 100;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
Fcn.dynamics = @(t,x,u)(robotDynamics(x,u,p));

% Fcn.pathObj = [];
% Fcn.pathObj = @(t,x,u)(ones(size(t)));
Fcn.pathObj = @(t,x,u)(sum(u.^2));
Fcn.bndObj = [];

Fcn.pathCst = @(t,x,u)pathCst(t,x,u,p);
Fcn.bndCst = @(t0,x0,tf,xf)bndCst(x0,xf,p);
Fcn.weights = ones(nGrid,1);
Fcn.weights([1,end]) = 0.5;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% X = [x;y;th;dx;dy;dth]
Xi = [0;0.1;0;0;0;0];
Xf = [-p.L;0.18;pi;0;0;0];
time = [0 duration];

if iter == 1
    guess = fcn_initialGuess(time,Xi,Xf,p);
elseif iter == 2
    guess.time = linspace(time(1),time(2),nGrid);
    guess.state = interp1(tSoln',xSoln',guess.time')';
    guess.control = interp1(tSoln',uSoln',guess.time')';
end
[zGuess, pack] = packDecVar(guess.time, guess.state, guess.control);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
B.ti.low = 0;
B.ti.upp = 0;
B.tf.low = duration;
B.tf.upp = duration;
% B.tf.low = duration;
% B.tf.upp = duration;

B.Xi.low = Xi;
B.Xi.upp = Xi;
B.Xf.low = Xf - 1e3 * [0;0;0;1;1;1];
B.Xf.upp = Xf + 1e3 * [0;0;0;1;1;1];

B.X.low = [-2*p.L;  0;  -pi/2;  -100; -100; -100];
B.X.upp = [p.L/2;   2;  pi*2;   100;  100;  100];

B.U.low = -maxForce * ones(p.nU,1);
B.U.upp = maxForce * ones(p.nU,1);

tLow = linspace(B.ti.low,B.tf.low,p.nGrid);
xLow = [B.Xi.low, B.X.low*ones(1,nGrid-2), B.Xf.low];
uLow = B.U.low * ones(1,nGrid);
zLow = packDecVar(tLow,xLow,uLow);

tUpp = linspace(B.ti.upp,B.tf.upp,p.nGrid);
xUpp = [B.Xi.upp, B.X.upp*ones(1,nGrid-2), B.Xf.upp];
uUpp = B.U.upp * ones(1,nGrid);
zUpp = packDecVar(tUpp,xUpp,uUpp);

%% Construct Probelm Structure
% Objective
Problem.objective = ...
    @(z)(myObjective(z,pack,Fcn.pathObj,Fcn.bndObj,Fcn.weights));
% Constraints
Problem.nonlcon = ...
    @(z)(myConstraints(z,pack,Fcn.dynamics,Fcn.pathCst,Fcn.bndCst));

Problem.x0 = zGuess;
Problem.lb = zLow;
Problem.ub = zUpp;
Problem.Aineq = [];
Problem.bineq = [];
Problem.Aeq = [];
Problem.beq = [];
Problem.options = optimset('Display','iter',...
                           'Outputfcn',@(x,y,z)outfun(x,y,z,pack,p),...
                           'MaxFunEvals',1e5);
Problem.solver = 'fmincon';

disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
disp(['Running OptimTraj, iteration ',num2str(iter)]);
[zSoln, objVal,exitFlag,output] = fmincon(Problem);
[tSoln,xSoln,uSoln] = unPackDecVar(zSoln,pack);


disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
disp(['Iteration ',num2str(iter),' Complete!']);
disp(['Solve time = ',num2str(toc),' sec']);

end

%%
animate(zSoln,pack,p,20)

%%
gen_figs(zSoln,pack,p)





