% trajectory optimization for single leg jumping
clear;clc

nGrid = 7;

p.nGrid = nGrid;
p.mass = 2;
p.g = 9.81;
p.l1 = 0.14;
p.l2 = 0.14;
p.nco = 4;
p.mu = 2;
p.maxLegExt = 0.22;
p.Tst = 0.15;
p.taumax = 10;
vin = [1;-2];
vout = [1;2];
p.vin = vin;
p.vout = vout;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
guess = fcn_init_guess(p);

% decision variables z = [Xi,co,q,dq,tau]
[zGuess,pack] = ...
    pack_dec_var(guess.time,guess.Xi,guess.co,guess.state,guess.control,p);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
B.Xi.low = guess.Xi - [0.1 0.1 4 4]';
B.Xi.upp = guess.Xi + [0.1 0.1 4 4]';
B.Xf.low = guess.Xf - [0.1 0.1 4 4]';
B.Xf.upp = guess.Xf + [0.1 0.1 4 4]';

B.co.low = -100 * ones(2,4);
B.co.upp = 100 * ones(2,4);

B.Q.low = [-pi*1.5 0 -5 -5]';
B.Q.upp = [-pi*0.3 pi 5 5]';

B.U.low = p.taumax * [-1 -1]';
B.U.upp = p.taumax * [1 1]';

tLow = linspace(0,p.Tst,nGrid);
XiLow = B.Xi.low;
coLow = B.co.low;
QLow = B.Q.low * ones(1,nGrid);
ULow = B.U.low * ones(1,nGrid);
zLow = pack_dec_var(tLow,XiLow,coLow,QLow,ULow,p);

tUpp = linspace(0,p.Tst,nGrid);
XiUpp = B.Xi.upp;
coUpp = B.co.upp;
QUpp = B.Q.upp * ones(1,nGrid);
UUpp = B.U.upp * ones(1,nGrid);
zUpp = pack_dec_var(tUpp,XiUpp,coUpp,QUpp,UUpp,p);

%% Construct Problem structure
tic
Problem.objective = @(z)(my_obj(z,pack,p));
Problem.nonlcon = @(z)(my_cst(z,pack,p));

Problem.x0 = zGuess;
Problem.lb = [];
Problem.ub = [];
Problem.Aineq = [];
Problem.bineq = [];
Problem.Aeq = [];
Problem.beq = [];
Problem.options = optimset('Display','iter',...
                           'MaxFunEvals',1e5);
%                            'Outputfcn',@(x,y,z)outfun(x,y,z,pack,p),...
Problem.solver = 'fmincon';

disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
[zSoln, objVal,exitFlag,output] = fmincon(Problem);
[tSoln,XiSoln,coSoln,QSoln,tauSoln] = unpack_dec_var(zSoln,pack);
disp(['Solve time = ',num2str(toc),' sec']);









