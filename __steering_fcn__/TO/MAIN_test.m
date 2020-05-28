% trajectory optimization for single leg jumping
clear;clc

nGrid = 4;

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









