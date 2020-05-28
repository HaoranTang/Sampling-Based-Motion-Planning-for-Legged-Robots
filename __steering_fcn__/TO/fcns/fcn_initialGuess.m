function guess = fcn_initialGuess(time,Xi,Xf,p)

nGrid = p.nGrid;

tSpan = time([1,end]);
guess.time = linspace(tSpan(1),tSpan(2),nGrid);



guess.state = interp1(tSpan',[Xi,Xf]',guess.time')';

Uguess = [0;p.m*p.g;0;p.m*p.g];
guess.control = interp1(tSpan',[Uguess,Uguess]',guess.time')';







